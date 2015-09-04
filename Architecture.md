Architecture Overview
====================

Todo: Visual diagrams

# Considerations
The sub should be able to...
- Change its trajectory based on new information at any time
- Respond to safely respond to an unrecoverable error
- Internally handle typical problems, without needing a restart
- Perform actions while moving
- Perform multiple non-conflicting actions at once 
    - (Move while pointing; Point and open gripper; etc)
- Navigate without landmarks safely for some distance
- Generate and operate on a traversability map

# Open Questions
- Can we ever make all three cameras work? Is it a bus bandwidth problem?