all: test_atlas.py

OPENHRP3_DIR = $(shell rospack find openhrp3_tutorials)

atlas.l: ../models/atlas.dae
	rosrun euscollada collada2eus $< $@ || echo "ok"

# make && ./test_atlas.py ; grep check_link /home/k-okada/.ros/test_results/openhrp3/TEST-test_atlas.xml to confirm joint order
test_atlas.py: Makefile atlas.l ${OPENHRP3_DIR}/test/gen-testcode.l
	rosrun euslisp irteusgl atlas.l ${OPENHRP3_DIR}/test/gen-testcode.l "(gen-testcode (atlas) '(:ltorso_lk :mtorso_lk :utorso_lk :l_clav_lk :l_scap_lk :l_uarm_lk :l_larm_lk :l_farm_lk :l_hand_lk :atlas_head_lk :center_bottom_led_frame_lk :center_top_led_frame_lk :head_imu_link_lk :hokuyo_link_lk :head_hokuyo_frame_lk :left_camera_frame_lk :left_camera_optical_frame_lk  :left_led_frame_lk :right_camera_frame_lk :right_camera_optical_frame_lk :right_led_frame_lk :r_clav_lk :r_scap_lk :r_uarm_lk :r_larm_lk :r_farm_lk :r_hand_lk :imu_link_lk :l_uglut_lk :l_lglut_lk :l_uleg_lk :l_lleg_lk :l_talus_lk :l_foot_lk :r_uglut_lk :r_lglut_lk :r_uleg_lk :r_lleg_lk :r_talus_lk :r_foot_lk) \"hrpsys_gazebo\" \"models/atlas.dae\")" "(exit)"
	chmod u+x $@

clean:
	rm -fr *.dae test_atlas.py


