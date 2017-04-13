dockerNode(image: 'uf-mil:subjugator') {
	stage("Checkout") {
		checkout scm
		sh '''
			ln -s `pwd` ~/mil_ws/src/
		'''
	}
	stage("Build") {
		sh '''
			source ~/.mil/milrc > /dev/null 2>&1
			cd $CATKIN_DIR/src
			git clone --recursive https://github.com/uf-mil/mil_common
			catkin_make -C $CATKIN_DIR -B
		'''
	}
	stage("Test") {
		sh '''
			source ~/.mil/milrc > /dev/null 2>&1
			source $CATKIN_DIR/devel/setup.bash > /dev/null 2>&1
			catkin_make -C $CATKIN_DIR run_tests
			ls $CATKIN_DIR/src
		'''
	}
}

