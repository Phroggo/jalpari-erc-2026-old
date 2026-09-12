"""Fast regression tests; run inside the competition image with unittest."""
import unittest
from types import SimpleNamespace
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from erc_solution.kinematics import Kinematics, transform, transport_path
from erc_solution.vision import objects, project, projected_box, row_for_height
from erc_solution.localization import WallLocalizer, world_points
from erc_solution.robot import Robot
from erc_solution.mission import Mission


class PlacementTests(unittest.TestCase):
    def fake_robot(self, contact_after=None, unreachable=False):
        robot = SimpleNamespace(book_id='target', bin_hits={'other': 20., 'target': 1.},
                                joints={'joint': 0.}, moves=[], checks=[])
        robot.state = lambda name: None
        robot.now = lambda: 10.
        robot.matrix = lambda *args: np.eye(4)
        def solve(goal, seed):
            if unreachable:
                raise RuntimeError('Unreachable')
            return seed
        robot.arm = SimpleNamespace(names=['joint'], solve=solve)
        robot.require_held = lambda *args: robot.checks.append(args)
        def move(goal, seconds):
            robot.moves.append((goal.copy(), seconds))
            if contact_after and len(robot.moves) == contact_after:
                robot.bin_hits['target'] = 10.5
        robot.arm_pose = move
        robot.wait = lambda seconds: None
        return robot

    def test_descent_is_bounded_and_ignores_old_or_other_book_contact(self):
        robot = self.fake_robot()
        point = np.array([.7, -.1, 1.13])
        Mission.lower_over_bin(robot, point)
        self.assertEqual(len(robot.moves), 3)
        heights = [point[2]] + [goal[2, 3] for goal, _ in robot.moves]
        np.testing.assert_allclose(np.diff(heights), [-.03] * 3)
        np.testing.assert_allclose(point, [.7, -.1, 1.13])
        np.testing.assert_allclose(robot.moves[-1][0][:3, 3], [.7, -.1, 1.04])
        self.assertTrue(all(seconds == 1. for _, seconds in robot.moves))

    def test_matching_contact_stops_further_descent(self):
        robot = self.fake_robot(contact_after=2)
        Mission.lower_over_bin(robot, np.array([.7, -.1, 1.13]))
        self.assertEqual(len(robot.moves), 2)

    def test_unreachable_path_does_not_start_descent(self):
        robot = self.fake_robot(unreachable=True)
        with self.assertRaises(RuntimeError):
            Mission.lower_over_bin(robot, np.array([.7, -.1, 1.13]))
        self.assertEqual(robot.moves, [])


class CarryTimingTests(unittest.TestCase):
    def test_torso_only_lift_skips_unneeded_arm_motion(self):
        fake = SimpleNamespace(height=1.0, checks=[], moves=[])
        fake.matrix = lambda *args: transform([.6, -.24, fake.height])
        fake.state = lambda name: None
        fake.require_held = lambda *args: fake.checks.append(args)
        fake.book_id = 'target'
        def move(*args, **kwargs):
            fake.moves.append(args)
            fake.height = 1.25
        fake.move_joints = move
        # No arm interface: attempting an extra arm motion fails this test.
        Mission.prepare_transport(fake)
        self.assertEqual(len(fake.moves), 1)
        self.assertEqual(fake.checks, [('target', 'after raising torso for transport')])

    def test_already_high_book_needs_no_torso_or_arm_motion(self):
        fake = SimpleNamespace(matrix=lambda *args: transform([.6, -.24, 1.25]))
        Mission.prepare_transport(fake)

    def robot(self, target):
        fake = SimpleNamespace(joints={'joint': 0.}, moves=[])
        fake.arm = SimpleNamespace(names=['joint'], solve=lambda goal, seed: [target])
        fake.matrix = lambda *args: np.eye(4)
        fake.move_joints = lambda *args, **kwargs: fake.moves.append((args, kwargs))
        return fake

    def test_small_step_is_quicker_but_large_turn_gets_more_time(self):
        for target, duration in ((.12, .7), (.60, 2.), (6., 20.)):
            fake = self.robot(target)
            Robot.arm_pose(fake, np.eye(4), .7, joint_speed=.30)
            args, kwargs = fake.moves[0]
            self.assertAlmostEqual(args[3], duration)
            self.assertGreater(kwargs['timeout'], duration)
            self.assertEqual(kwargs['tolerance'], .025)

    def test_other_arm_moves_keep_requested_duration(self):
        fake = self.robot(.6)
        Robot.arm_pose(fake, np.eye(4), 1.8)
        self.assertEqual(fake.moves[0][0][3], 1.8)

    def test_invalid_speed_never_commands_motion(self):
        for speed in (0., -1., float('nan'), float('inf')):
            fake = self.robot(.1)
            with self.assertRaises(ValueError):
                Robot.arm_pose(fake, np.eye(4), joint_speed=speed)
            self.assertEqual(fake.moves, [])

    def test_tool_alignment_check_is_still_enforced(self):
        fake = self.robot(.1)
        with self.assertRaisesRegex(RuntimeError, 'Measured tool misalignment'):
            Robot.arm_pose(fake, transform([.1, 0., 0.]), .7, joint_speed=.30)


class ContactTests(unittest.TestCase):
    def test_gripper_body_is_not_a_second_finger(self):
        fake = SimpleNamespace(now=lambda:10.,stamp=Robot.stamp,contacts={})
        book = SimpleNamespace(name='book_a::book_base_link::collision')
        def message(link):
            return SimpleNamespace(
                header=SimpleNamespace(stamp=SimpleNamespace(sec=9,nanosec=900000000)),
                contacts=[SimpleNamespace(collision1=SimpleNamespace(
                    name=f'tiago_pro::{link}::collision'),collision2=book)])
        Robot.on_contacts(fake,message('gripper_right_fingertip_left_link'))
        Robot.on_contacts(fake,message('gripper_right_base_link'))
        self.assertEqual(Robot.held(fake),set())
        Robot.on_contacts(fake,message('gripper_right_fingertip_right_link'))
        self.assertEqual(Robot.held(fake),{'book_a'})
        self.assertEqual(set(fake.contacts.values()),{9.9})

    def test_stale_or_single_sided_contacts_are_not_a_hold(self):
        fake = SimpleNamespace(now=lambda:10.,contacts={
            ('book_a','gripper_right_fingertip_left_link'):9.9,
            ('book_a','gripper_right_fingertip_right_link'):9.0})
        self.assertEqual(Robot.held(fake),set())
        fake.contacts[('book_a','gripper_right_fingertip_right_link')] = 9.9
        self.assertEqual(Robot.held(fake),{'book_a'})

    def test_bounded_contact_recheck_handles_gap_not_loss(self):
        for recovered in (True,False):
            fake = SimpleNamespace(t=0.,cmd=SimpleNamespace(publish=lambda msg:None))
            fake.now = lambda:fake.t
            fake.tick = lambda:setattr(fake,'t',fake.t+.1)
            fake.held = lambda:{'book_a'} if recovered and fake.t>.7 else set()
            if recovered:
                Robot.require_held(fake,'book_a','in test')
                self.assertGreaterEqual(fake.t,.95)
            else:
                with self.assertRaises(RuntimeError):
                    Robot.require_held(fake,'book_a','in test')


class PerceptionTests(unittest.TestCase):
    def test_column_box_is_projected_and_clipped_to_live_image(self):
        points = [[-.5,-.5,1],[.5,.5,1]]
        self.assertEqual(projected_box(points,(100,100,50,50),(100,100,3)),(0,0,99,99))
        self.assertIsNone(projected_box([[0,0,-1]],(100,100,50,50),(100,100,3)))

    def test_depth_rejects_missing_and_edges(self):
        d = np.ones((40,40))
        k = (100,100,20,20)
        np.testing.assert_allclose(project(d,k,20,20),[0,0,1])
        d[16:25,18:23] = np.nan
        self.assertIsNone(project(d,k,20,20))
        self.assertIsNone(project(d,k,-1,0))

    def test_two_pixel_spine_survives(self):
        rgb = np.zeros((100,100,3),np.uint8)
        rgb[20:45,40:42] = [0,0,255]
        found = objects(rgb,np.ones((100,100)),(100,100,50,50),'red')
        self.assertEqual(len(found),1)

    def test_clipped_book_centroid_is_not_a_grasp_target(self):
        rgb = np.zeros((100,100,3),np.uint8)
        rgb[:25,40:43] = [0,0,255]
        self.assertEqual(objects(rgb,np.ones((100,100)),(100,100,50,50),'red'),[])

    def test_row_is_not_rank_of_visible_books(self):
        self.assertEqual(row_for_height(.917),3)
        self.assertEqual(row_for_height(.587),4)
        self.assertIsNone(row_for_height(2.26))

    def test_red_bin_is_not_a_red_book(self):
        rgb = np.zeros((100,100,3),np.uint8)
        rgb[30:50,20:80] = [0,0,255]
        d = np.ones((100,100))
        self.assertEqual(len(objects(rgb,d,(100,100,50,50),'red',True)),1)
        self.assertEqual(len(objects(rgb,d,(100,100,50,50),'red')),0)

    def test_hollow_bin_centroid_need_not_have_depth(self):
        rgb = np.zeros((100,100,3),np.uint8)
        rgb[30:65,20:80] = [0,0,255]
        rgb[34:61,24:76] = 0
        depth = np.full((100,100),np.nan)
        depth[rgb[:,:,2]>0] = 1.
        self.assertEqual(len(objects(rgb,depth,(100,100,50,50),'red',True)),1)

    def test_bin_may_appear_taller_than_wide(self):
        rgb = np.zeros((100,100,3),np.uint8)
        rgb[15:85,25:75] = [0,0,255]
        self.assertEqual(len(objects(rgb,np.ones((100,100)),
                                     (100,100,50,50),'red',True)),1)

    def test_bin_height_gate_excludes_carried_red_book(self):
        rgb = np.zeros((100,100,3),np.uint8)
        rgb[20:55,60:75] = [0,0,255]
        rgb[50:70,20:80] = [0,0,255]
        depth = np.full((100,100),.85)
        depth[20:50,60:75] = 1.6
        found = objects(rgb,depth,(100,100,50,50),'red',True,np.eye(4))
        self.assertEqual(len(found),1)
        self.assertGreaterEqual(found[0].box[1],50)


class KinematicsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from ament_index_python.packages import get_package_share_directory
        cls.path = Path(get_package_share_directory('erc_description'))/'urdf/tiago_pro.urdf'
        cls.arm = Kinematics(cls.path)

    def test_fk_ik_roundtrip(self):
        q = np.clip([-.5,-.6,0,1.4,0,-.8,0],self.arm.lower,self.arm.upper)
        goal = self.arm.fk(q)
        solved = self.arm.solve(goal,q+.01)
        np.testing.assert_allclose(self.arm.fk(solved),goal,atol=.002)

    def test_tool_x_is_physical_approach(self):
        base = Kinematics(self.path,tip='gripper_right_base_link')
        q = np.clip(np.zeros(7),self.arm.lower,self.arm.upper)
        # URDF grasping frame is rotated -pi/2 around Y relative to gripper base.
        np.testing.assert_allclose(self.arm.fk(q)[:3,0],base.fk(q)[:3,2],atol=.001)

    def test_unreachable_is_rejected(self):
        with self.assertRaises(RuntimeError):
            self.arm.solve(transform([10,10,10]),np.zeros(7))

    def test_compact_posture(self):
        waypoint = self.arm.startup_waypoint(np.zeros(7))
        q = self.arm.compact(waypoint)
        self.assertLess(self.arm.points(q)[:,0].max(),.40)
        self.assertGreater(min(self.arm.points(waypoint+(q-waypoint)*t)[:,2].min()
                               for t in np.linspace(0,1,41)),-.70)
        left = Kinematics(self.path,tip='gripper_left_grasping_link')
        waypoint = left.startup_waypoint(np.zeros(7),side=1)
        q = left.compact(waypoint,side=1)
        self.assertLess(left.points(q)[:,0].max(),.40)
        self.assertGreater(min(left.points(waypoint+(q-waypoint)*t)[:,2].min()
                               for t in np.linspace(0,1,41)),-.70)

    def test_all_four_nominal_grasp_paths(self):
        rows = [(1.577,.345,.55),(1.247,.20,.72),(.917,0.,.75),(.587,0.,.85)]
        for height,torso,reach in rows:
            q = self.arm.compact(np.zeros(7))
            for dx,dz in ((-.09,0),(.06,0),(.06,.025),(-.14,.025)):
                with self.subTest(height=height,dx=dx,dz=dz):
                    goal = transform([reach+dx+.0245,-.24,height+dz-.8457-torso])
                    q = self.arm.solve(goal,q)

    def test_pi_rotation_is_finite(self):
        angle = Rotation.from_matrix(transform(rpy=[np.pi,0,0])[:3,:3]).as_rotvec()
        self.assertAlmostEqual(np.linalg.norm(angle),np.pi)

    def test_lower_row_transport_paths(self):
        for height,torso,reach in ((.917,0.,.75),(.587,0.,.85)):
            q = self.arm.compact(np.zeros(7))
            start = np.array([reach-.14+.0245,-.24,height+.025-.8457-torso])
            q = self.arm.solve(transform(start),q)
            base_from_root = transform([-.0245,0.,.8457+.345])
            for goal in transport_path((base_from_root @ np.r_[start,1])[:3]):
                q = self.arm.solve(np.linalg.inv(base_from_root) @ goal,q)


class LocalizationTests(unittest.TestCase):
    def test_camera_plane_resolves_square_room(self):
        pose = np.array([-1.8,-1.6,.9])
        y = np.linspace(-4.5,4.5,100)
        x = np.linspace(-3.5,5.5,100)
        world = np.r_[np.c_[np.full(100,-3.975),y],np.c_[np.full(100,5.975),y],
                      np.c_[x,np.full(100,-4.975)],np.c_[x,np.full(100,4.975)]]
        c,s = np.cos(pose[2]),np.sin(pose[2])
        r = np.array([[c,-s],[s,c]])
        points = (world-pose[:2]) @ r
        localizer = WallLocalizer()
        localizer.points = points
        marker = np.r_[np.array([2.755,0])-pose[:2],2.26]
        marker[:2] = marker[:2] @ r
        normal = np.array([c,-s,0.])
        self.assertTrue(localizer.initialize_shelf(marker,normal))
        np.testing.assert_allclose(localizer.pose,pose,atol=.005)

    def test_missing_scan_rejected(self):
        localizer = WallLocalizer()
        self.assertIsNone(localizer.update(np.zeros((2,2))))


if __name__ == '__main__':
    unittest.main()
