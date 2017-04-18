dockerNode(image: 'uf-mil:ros_alarms') {
	stage("Checkout") {
		checkout scm
		sh '''
			git submodule update --init --recursive
			ln -s `pwd` ~/catkin_ws/src/ros_alarms
			git clone --recursive https://github.com/txros/txros ~/catkin_ws/src/txros
		'''
	}
	stage("Build") {
		sh '''
			source /opt/ros/kinetic/setup.bash > /dev/null 2>&1
			catkin_make -C ~/catkin_ws -B
		'''
	}
	stage("Test") {
		sh '''
			source /opt/ros/kinetic/setup.bash > /dev/null 2>&1
			source ~/catkin_ws/devel/setup.bash > /dev/null 2>&1
			catkin_make -C ~/catkin_ws run_tests
			catkin_test_results ~/catkin_ws/build/test_results --verbose 
		'''
	}
}
