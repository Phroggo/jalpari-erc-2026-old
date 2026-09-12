# Submission verification — Jalpari, final_code, 11 September 2026

Audited baseline: `2ebfc9955116252fd2fc0cf10053e34aee0282d0`.
Subsequent cleanup removes three unused training scripts and updates documentation;
the active mission, launch interface, model and motion algorithm are unchanged.

## Official simulator

Runtime organizer source and Dockerfile match official main
`68e175fada5b8115b0f7bbb0e9b2b08a96504f22`, using Ubuntu 22.04, ROS 2 Humble
and Gazebo Harmonic. Compose adds an evidence-directory mount.
The organizer [merged the gripper updates into main for continued development](https://github.com/dfl-rlab/erc_sim_2026/issues/2#issuecomment-5582821341).
The [official README](https://github.com/dfl-rlab/erc_sim_2026/blob/68e175fada5b8115b0f7bbb0e9b2b08a96504f22/README.md)
announces 2 cm book spines. The latest tag v1.0.3 predates these updates; the
supported development baseline is current main. Check for later organizer updates
before submitting. The solution requires no robot-model or physics modifications.

## Vision and onboard navigation

The rulebook lists onboard LiDAR and requires navigation using onboard sensors
and vision. Section 1.5 discusses “entirely vision-based” localization in the
context of identifying randomized column numbers. Our reading is that target
column identification must use vision, not that onboard LiDAR navigation is
prohibited. No explicit LiDAR-navigation ban was found in the supplied rules.

Live RGB-D identifies the marker digit, requested book and bin. LiDAR scan
matching against fixed arena walls tracks the robot, initialized with the observed
shelf plane. Known start-zone and bin-search geometry are priors; they do not
select the randomized column or replace live target observations. The runtime
does not read simulator object poses or teleport/attach objects.

This is consistent with the organizer's guidance to
[identify shelves visually after reorientation](https://github.com/dfl-rlab/erc_sim_2026/issues/5#issuecomment-5405434980)
and [configure the supplied navigation tools](https://github.com/dfl-rlab/erc_sim_2026/issues/4#issuecomment-5395015649).
LiDAR navigation is not treated as a submission blocker. This explanation is a
reading of the published rules, not a claim of individual organizer certification.

## Fresh-container checks

The unchanged official Dockerfile was built using cached Docker build layers.
An isolated fresh container, erc_aaron_submission_audit, used networking disabled
and ROS domain 85. Submitted src was copied into an empty workspace without prior
build/install artifacts or development-container modifications.

- All 33 packages built successfully in 15.5 seconds.
- All 21 regression tests passed in 1.726 seconds.
- Installed launch argument discovery passed for column, color and images_dir.
- Mission initialization loaded the model, URDF and runtime imports.
- Default evidence path resolved to /opt/erc_ws/erc_images with symlink install.

No additional apt/pip packages were needed. These are installation/initialization
checks on one host, not a new end-to-end trial or cross-machine verification.

## Evidence and limitations

The runtime matches the earlier tested Gloria implementation. Recovered stress
JSONL records contain 25 terminal successes, one failure and one incomplete run.
Early debugging invocations overlapped; this is not a controlled reliability study.
Successful node-relative wall times average 247.80 seconds, range 186.81–327.13.
Ten of twenty column/color input combinations were exercised across randomized
layouts. Raw evidence is preserved in the local submission audit outside this repo.

Host stress CSV zeros were a logging error. JSONL wall_elapsed begins at node
creation, not launch invocation; sim_time is an absolute simulator reading.
Use an actual launch-to-contact wall timer for scoring.

Required Int32 identification topics, live bounding boxes and visible timestamps
are implemented. Only the right arm retrieves/delivers; the left parks at startup.
Success checks matching held-book contact with the bin after release, but does
not independently establish maximum score, zero collisions or gentle placement.
The gripper opens above the bin; report drop placement unless gentle placement
is demonstrated. Full collision scores have not been reconstructed.

Direct navigation stops on obstacles without replanning. IK checks reachability,
not full-mesh collision clearance. Most failures abort rather than recover/retry.

Unused training/build_map.py, training/_graspseq.py and training/debug_books.py
have been removed from the submission tree. Git history retains deleted files.
The report, video, submission form and team/contributor eligibility remain separate
deliverables/checks. Preserve contributor attribution.
