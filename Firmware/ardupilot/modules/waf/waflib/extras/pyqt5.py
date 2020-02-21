#!/usr/bin/env python
# encoding: utf-8
# Federico Pellegrin, 2016 (fedepell) adapted for Python

"""
This tool helps with finding Python Qt5 tools and libraries,
and provides translation from QT5 files to Python code.

The following snippet illustrates the tool usage::

	def options(opt):
		opt.load('py pyqt5')

	def configure(conf):
		conf.load('py pyqt5')

	def build(bld):
		bld(
			features = 'py pyqt5',
			source   = 'main.py textures.qrc aboutDialog.ui',
		)

Here, the UI description and resource files will be processed
to generate code.

Usage
=====

Load the "pyqt5" tool.

Add into the sources list also the qrc resources files or ui5
definition files and they will be translated into python code
with the system tools (PyQt5 or pyside2 are searched in this
order) and then compiled
"""

try:
	from xml.sax import make_parser
	from xml.sax.handler import ContentHandler
except ImportError:
	has_xml = False
	ContentHandler = object
else:
	has_xml = True

import os
from waflib.Tools import python
from waflib import Task
from waflib.TaskGen import feature, extension
from waflib.Configure import conf
from waflib import Logs

EXT_RCC = ['.qrc']
"""
File extension for the resource (.qrc) files
"""

EXT_UI  = ['.ui']
"""
File extension for the user interface (.ui) files
"""


class XMLHandler(ContentHandler):
	"""
	Parses ``.qrc`` files
	"""
	def __init__(self):
		self.buf = []
		self.files = []
	def startElement(self, name, attrs):
		if name == 'file':
			self.buf = []
	def endElement(self, name):
		if name == 'file':
			self.files.append(str(''.join(self.buf)))
	def characters(self, cars):
		self.buf.append(cars)

@extension(*EXT_RCC)
def create_pyrcc_task(self, node):
	"Creates rcc and py task for ``.qrc`` files"
	rcnode = node.change_ext('.py')
	self.create_task('pyrcc', node, rcnode)
	if getattr(self, 'install_from', None):
		self.install_from = self.install_from.get_bld()
	else:
		self.install_from = self.path.get_bld()
	self.install_path = getattr(self, 'install_path', '${PYTHONDIR}')
	self.process_py(rcnode)

@extension(*EXT_UI)
def create_pyuic_task(self, node):
	"Create uic tasks and py for user interface ``.ui`` definition files"
	uinode = node.change_ext('.py')
	self.create_task('ui5py', node, uinode)
	if getattr(self, 'install_from', None):
		self.install_from = self.install_from.get_bld()
	else:
		self.install_from = self.path.get_bld()
	self.install_path = getattr(self, 'install_path', '${PYTHONDIR}')
	self.process_py(uinode)

@extension('.ts')
def add_pylang(self, node):
	"""Adds all the .ts file into ``self.lang``"""
	self.lang = self.to_list(getattr(self, 'lang', [])) + [node]

@feature('pyqt5')
def apply_pyqt5(self):
	"""
	The additional parameters are:

	:param lang: list of translation files (\*.ts) to process
	:type lang: list of :py:class:`waflib.Node.Node` or string without the .ts extension
	:param langname: if given, transform the \*.ts files into a .qrc files to include in the binary file
	:type langname: :py:class:`waflib.Node.Node` or string without the .qrc extension
	"""
	if getattr(self, 'lang', None):
		qmtasks = []
		for x in self.to_list(self.lang):
			if isinstance(x, str):
				x = self.path.find_resource(x + '.ts')
			qmtasks.append(self.create_task('ts2qm', x, x.change_ext('.qm')))


		if getattr(self, 'langname', None):
			qmnodes = [k.outputs[0] for k in qmtasks]
			rcnode = self.langname
			if isinstance(rcnode, str):
				rcnode = self.path.find_or_declare(rcnode + '.qrc')
			t = self.create_task('qm2rcc', qmnodes, rcnode)
			create_pyrcc_task(self, t.outputs[0])

class pyrcc(Task.Task):
	"""
	Processes ``.qrc`` files
	"""
	color   = 'BLUE'
	run_str = '${QT_PYRCC} ${SRC} -o ${TGT}'
	ext_out = ['.py']

	def rcname(self):
		return os.path.splitext(self.inputs[0].name)[0]

	def scan(self):
		"""Parse the *.qrc* files"""
		if not has_xml:
			Logs.error('No xml.sax support was found, rcc dependencies will be incomplete!')
			return ([], [])

		parser = make_parser()
		curHandler = XMLHandler()
		parser.setContentHandler(curHandler)
		fi = open(self.inputs[0].abspath(), 'r')
		try:
			parser.parse(fi)
		finally:
			fi.close()

		nodes = []
		names = []
		root = self.inputs[0].parent
		for x in curHandler.files:
			nd = root.find_resource(x)
			if nd: nodes.append(nd)
			else: names.append(x)
		return (nodes, names)


class ui5py(Task.Task):
	"""
	Processes ``.ui`` files for python
	"""
	color   = 'BLUE'
	run_str = '${QT_PYUIC} ${SRC} -o ${TGT}'
	ext_out = ['.py']

class ts2qm(Task.Task):
	"""
	Generates ``.qm`` files from ``.ts`` files
	"""
	color   = 'BLUE'
	run_str = '${QT_LRELEASE} ${QT_LRELEASE_FLAGS} ${SRC} -qm ${TGT}'

class qm2rcc(Task.Task):
	"""
	Generates ``.qrc`` files from ``.qm`` files
	"""
	color = 'BLUE'
	after = 'ts2qm'
	def run(self):
		"""Create a qrc file including the inputs"""
		txt = '\n'.join(['<file>%s</file>' % k.path_from(self.outputs[0].parent) for k in self.inputs])
		code = '<!DOCTYPE RCC><RCC version="1.0">\n<qresource>\n%s\n</qresource>\n</RCC>' % txt
		self.outputs[0].write(code)

def configure(self):
	self.find_pyqt5_binaries()

	# warn about this during the configuration too
	if not has_xml:
		Logs.error('No xml.sax support was found, rcc dependencies will be incomplete!')

@conf
def find_pyqt5_binaries(self):
	"""
	Detects PyQt5 or pyside2 programs such as pyuic5/pyside2-uic, pyrcc5/pyside2-rcc
	"""
	env = self.env

	self.find_program(['pyuic5','pyside2-uic'], var='QT_PYUIC')
	if not env.QT_PYUIC:
		self.fatal('cannot find the uic compiler for python for qt5')

	self.find_program(['pyrcc5','pyside2-rcc'], var='QT_PYRCC')
	if not env.QT_PYUIC:
		self.fatal('cannot find the rcc compiler for python for qt5')

	self.find_program(['pylupdate5', 'pyside2-lupdate'], var='QT_PYLUPDATE')
	self.find_program(['lrelease-qt5', 'lrelease'], var='QT_LRELEASE')

