#! /usr/bin/env python
# encoding: utf-8
# Thomas Nagy, 2016 (ita)

import sys, traceback, base64
try:
	import cPickle
except ImportError:
	import pickle as cPickle

try:
	import subprocess32 as subprocess
except ImportError:
	import subprocess

def run():
	txt = sys.stdin.readline().strip()
	if not txt:
		# parent process probably ended
		sys.exit(1)
	[cmd, kwargs, cargs] = cPickle.loads(base64.b64decode(txt))
	cargs = cargs or {}

	ret = 1
	out, err, ex, trace = (None, None, None, None)
	try:
		proc = subprocess.Popen(cmd, **kwargs)
		out, err = proc.communicate(**cargs)
		ret = proc.returncode
	except (OSError, ValueError, Exception) as e:
		exc_type, exc_value, tb = sys.exc_info()
		exc_lines = traceback.format_exception(exc_type, exc_value, tb)
		trace = str(cmd) + '\n' + ''.join(exc_lines)
		ex = e.__class__.__name__

	# it is just text so maybe we do not need to pickle()
	tmp = [ret, out, err, ex, trace]
	obj = base64.b64encode(cPickle.dumps(tmp))
	sys.stdout.write(obj.decode())
	sys.stdout.write('\n')
	sys.stdout.flush()

while 1:
	try:
		run()
	except KeyboardInterrupt:
		break

