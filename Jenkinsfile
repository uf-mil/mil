dockerNode(image: 'uf-mil:mil_common') {
	stage("Checkout") {
		checkout scm
		sh '''
			source ~/.mil/milrc > /dev/null 2>&1
			git submodule update --init --recursive
			ln -s `pwd` $CATKIN_DIR/src/mil_common
			ln -s $MIL_CONFIG_DIR/bvtsdk $CATKIN_DIR/src/mil_common/drivers/mil_blueview_driver/bvtsdk
		'''
	}
	stage("Build") {
		sh '''
			source ~/.mil/milrc > /dev/null 2>&1
			catkin_make -C $CATKIN_DIR -B
		'''
	}
	stage("Test") {
		sh '''
			source ~/.mil/milrc > /dev/null 2>&1
			source $CATKIN_DIR/devel/setup.bash > /dev/null 2>&1
			catkin_make -C $CATKIN_DIR run_tests
			catkin_test_results $CATKIN_DIR/build/test_results --verbose
		'''
	}
}
