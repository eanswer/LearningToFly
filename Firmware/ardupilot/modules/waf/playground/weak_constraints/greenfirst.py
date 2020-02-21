#! /usr/bin/env python
# encoding: utf-8

from waflib import Task, Runner

# amount of tasks to wait before trying to mark tasks as done
# see Runner.py for details
# setting the gap to a low value may seriously degrade performance
Runner.GAP = 1

# shrinking sets of dependencies provided to know quickly which yelow tasks are ready
reverse_map = {}

old = Task.set_file_constraints
def green_first(lst):
	# weak order constraint based on lexicographic order:
	# green before pink before yellow
	lst.sort(cmp=lambda x, y: cmp(x.__class__.__name__, y.__class__.__name__))

	# call the previous method to order the tasks by file dependencies
	old(lst)

	# shrinking sets preparation
	for tsk in lst:
		for k in tsk.run_after:
			try:
				reverse_map[k].add(tsk)
			except KeyError:
				reverse_map[k] = set([tsk])
Task.set_file_constraints = green_first

def get_out(self):
	# this is called whenever a task has finished executing
	tsk = self.prev_get_out()

	for x in reverse_map.get(tsk, []):
		# remote this task from the dependencies of other tasks
		# this is a minor optimization in general
		# but here we use this to know which tasks are ready to run
		x.run_after.remove(tsk)

	if tsk.__class__.__name__ == 'green':
		# whenever a green task is done it may be time to put
		# one or more yellow tasks in front
		# some additional optimization can be performed if exactly one
		# yellow task can be added to avoid whole list traversal
		def extract_yellow(lst):
			# return the yellow tasks that we can run immediately
			front = []
			lst.reverse()
			for tsk in lst:
				if tsk.__class__.__name__ == 'yellow' and not tsk.run_after:
					#print("found one yellow task to run first")
					lst.remove(tsk)
					front.append(tsk)
			lst.reverse()
			return front
		# this sets the order again
		self.outstanding = extract_yellow(self.outstanding) + extract_yellow(self.frozen) + self.outstanding
	return tsk
Runner.Parallel.prev_get_out = Runner.Parallel.get_out
Runner.Parallel.get_out      = get_out

