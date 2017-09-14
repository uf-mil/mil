dockerNode(image: 'uf-mil:navigator') {
	stage("Checkout") {
		checkout scm
		sh '''
			source ~/.mil/milrc > /dev/null 2>&1
			git submodule update --init --recursive
			ln -s $PWD $CATKIN_DIR/src/NaviGator
			git clone --recursive https://github.com/uf-mil/mil_common $CATKIN_DIR/src/mil_common
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
	stage("Format") {
		sh '''
			if [[ ! -z "$(python2.7 -m flake8 --ignore E731 --max-line-length=120 --exclude=__init__.py .)" ]]; then
				echo "The preceding Python files are not formatted correctly"
				exit 1
			fi
		'''
		sh '''
			source /opt/ros/kinetic/setup.bash > /dev/null 2>&1
			wget -O ~/.clang-format https://raw.githubusercontent.com/uf-mil/installer/master/.clang-format
			for FILE in $(find -path ./deprecated -prune -o -regex ".*/.*\\.\\(c\\|cc\\|cpp\\|h\\|hpp\\)$"); do
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
