dockerNode(image: 'uf-mil:ros_alarms') {
	stage("Checkout") {
		checkout scm
		sh '''
			git submodule update --init --recursive
			ln -s $PWD ~/catkin_ws/src/ros_alarms
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
	stage("Format") {
		sh '''
			if [[ ! -z "$(python2.7 -m flake8 --max-line-length=120 --exclude=__init__.py .)" ]]; then
				echo "The preceding Python following files are not formatted correctly"
				exit 1
			fi
		'''
		sh '''
			source /opt/ros/kinetic/setup.bash > /dev/null 2>&1
			wget -O ~/.clang-format https://raw.githubusercontent.com/uf-mil/installer/master/.clang-format
			for FILE in $(find . -name '*.h' -or -name '*.hpp' -or -name '*.cpp'); do
				if [ ! -z "$(clang-format-3.8 -style=file -output-replacements-xml $FILE | grep '<replacement ')" ]; then
					FILES+=( "$FILE" )
				fi
			done
			if (( ${#FILES[@]} > 0 )); then
				echo "The C++ following files are not formatted correctly: ${FILES[@]}"
				exit 1
			fi
		'''
	}
}
