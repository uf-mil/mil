# PCODAR
pcodardelete() {
	rosservice call /database/requests "name: '$1'
cmd: 'delete'"
}
