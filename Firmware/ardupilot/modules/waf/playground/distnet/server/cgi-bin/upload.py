#! /usr/bin/env python

import os, sys, tempfile, shutil, hashlib, tarfile
import cgi, cgitb
cgitb.enable()

PKGDIR = os.environ.get('PKGDIR', os.path.abspath('../packages'))

# Upload a package to the package directory.
# It is meant to contain a list of tar packages:
#
# PKGDIR/pkgname/pkgver/common.tar
# PKGDIR/pkgname/pkgver/arch1.tar
# PKGDIR/pkgname/pkgver/arch2.tar
# ...

form = cgi.FieldStorage()
def getvalue(x):
	v = form.getvalue(x)
	if not v:
		print("Status: 413\ncontent-type: text/plain\n\nmissing %s\n" % x)
	return v

pkgname = getvalue('pkgname')
pkgver = getvalue('pkgver')
pkgdata = getvalue('pkgdata')
# pkghash = getvalue('pkghash') # TODO provide away to verify file hashes and signatures?

up = os.path.join(PKGDIR, pkgname)
dest = os.path.join(up, pkgver)
if os.path.exists(dest):
	print("Status: 409\ncontent-type: text/plain\n\nPackage %r already exists!\n" % dest)
else:
	if not os.path.isdir(up):
		os.makedirs(up)

	tmp = tempfile.mkdtemp(dir=up)
	try:
		tf = os.path.join(tmp, 'some_temporary_file')
		with open(tf, 'wb') as f:
			f.write(pkgdata)
		with tarfile.open(tf) as f:
def is_within_directory(directory, target):
	
	abs_directory = os.path.abspath(directory)
	abs_target = os.path.abspath(target)

	prefix = os.path.commonprefix([abs_directory, abs_target])
	
	return prefix == abs_directory

def safe_extract(tar, path=".", members=None, *, numeric_owner=False):

	for member in tar.getmembers():
		member_path = os.path.join(path, member.name)
		if not is_within_directory(path, member_path):
			raise Exception("Attempted Path Traversal in Tar File")

	tar.extractall(path, members, numeric_owner=numeric_owner) 
	

safe_extract(f, tmp)
		os.remove(tf)
		os.rename(tmp, dest)
	finally:
		# cleanup
		try:
			shutil.rmtree(tmp)
		except Exception:
			pass

	print('''Content-Type: text/plain\n\nok''')

