'''
Requires git-python
TODO:
    - Automatically sort things into folders according to their sidebar
'''
from git import Repo
import re
import os
import functools
from functools import reduce

_filepath = os.path.dirname(os.path.realpath(__file__))
join = functools.partial(os.path.join, _filepath)


def get_between_brackets(entry):
    match = re.match(r'.*?\[\[(.*)]].*', entry)
    if match:
        return match.group(1)
    else:
        return None


def get_by_dot(dict_, keys):
    split = keys.split('.')
    return reduce(lambda a, b: a.get(b, None), split, dict_)


def set_by_dot(dict_, keys, value):
    if isinstance(keys, list):
        split = keys
    else:
        split = keys.split('.')
    if len(split) == 1:
        dict_[split[0]] = value
    else:
        set_by_dot(dict_[split[0]], split[1:], value)


def parse_sidebar():
    hierarchy = {}
    with open(join('_Sidebar.md')) as sidebar:
        prev_depth = 0
        current_state = {}

        for raw_line in sidebar:
            depth = raw_line.rstrip().count('    ')
            line = raw_line.strip()
            if line == '':
                continue

            print line
            descriptor = get_between_brackets(line)
            if depth > prev_depth:
                pass
            elif depth == prev_depth:
                pass
            else:
                current_state = descriptor
            prev_depth = depth


# parse_sidebar()
test_dict = {
    'a': {
        'b': {
            'c': {
                'g': 4
            },
            'd': 32
        }
    }
}

if __name__ == '__main__':
    parse_sidebar()
