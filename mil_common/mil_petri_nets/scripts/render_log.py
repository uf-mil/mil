#!/usr/bin/env python3
import graphviz
import sys
import tempfile


HEADER = '[Next State of ' + sys.argv[1] + '] :\n'
tmp = tempfile.NamedTemporaryFile()

log = open(sys.argv[1], 'r').read()
log = log[log.index(HEADER) + len(HEADER):]
while log.find(HEADER) != -1:
    graph = log[:log.find(HEADER)]
    graph = graphviz.Source(graph)
    log = log[log.index(HEADER) + len(HEADER):]
    graph.render(tmp.name, view=True)
    input()
tmp.close()
