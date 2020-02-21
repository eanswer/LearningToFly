## ABOUT WAF

Waf is a Python-based framework for configuring, compiling and installing applications. Here are perhaps the most important features of Waf:

  * *Automatic build order*: the build order is computed from input and output files, among others
  * *Automatic dependencies*: tasks to execute are detected by hashing files and commands
  * *Performance*: tasks are executed in parallel automatically, the startup time is meant to be fast (separation between configuration and build)
  * *Flexibility*: new commands and tasks can be added very easily through subclassing, bottlenecks for specific builds can be eliminated through dynamic method replacement
  * *Extensibility*: though many programming languages and compilers are already supported by default, many others are available as extensions
  * *IDE support*: Eclipse, Visual Studio and Xcode project generators (waflib/extras/)
  * *Documentation*: the application is based on a robust model documented in [The Waf Book](https://waf.io/book/) and in the [API docs](https://waf.io/apidocs/)
  * *Python compatibility*: cPython 2.5 to 3.4, Jython 2.5, IronPython, and Pypy

Waf is used in particular by innovative companies such as [Avalanche Studios](http://www.avalanchestudios.se) and by open-source projects such as [RTEMS](https://www.rtems.org/). Learn more about Waf by reading [The Waf Book](https://waf.io/book/).

For researchers and build system writers, Waf also provides a framework for creating [custom build systems](https://github.com/waf-project/waf/tree/master/build_system_kit) and [package distribution systems](https://github.com/waf-project/waf/tree/master/playground/distnet/README.rst).

Download the project from our page on [waf.io](https://waf.io/) or from a mirror on [freehackers.org](http://www.freehackers.org/~tnagy/release/), consult the [manual](https://waf.io/book/), the [API documentation](https://waf.io/apidocs/) and the [showcases](https://github.com/waf-project/waf/tree/master/demos) and [experiments](https://github.com/waf-project/waf/tree/master/playground).

## HOW TO CREATE THE WAF SCRIPT

Python >= 2.6 is required to generate the waf script, and the resulting file can then run on Python 2.5.
Just execute:
```sh
$ ./waf-light configure build
```
Or, if you have several python versions installed:
```sh
$ python3 ./waf-light configure build
```

The Waf tools in waflib/extras are not added to the waf script. To add
some of them, use the --tools switch:
```sh
$ ./waf-light --tools=compat15,swig
```

To add a tool that does not exist in the folder extras, pass an absolute path, and
to customize the initialization, pass the parameter 'prelude'. Here is for example
how to create a waf file using the compat15 module:
```sh
$ ./waf-light --tools=compat15 --prelude=$'\tfrom waflib.extras import compat15\n'
```

Any kind of initialization is possible, though one may prefer the build system kit (folder build\_system\_kit):
```sh
$ ./waf-light --make-waf --tools=compat15,/comp/waf/aba.py --prelude=$'\tfrom waflib.extras import compat15\n\tprint("ok")'
```

Or, to avoid regenerating the waf file all the time, just set the `WAFDIR` environment variable to the directory containing "waflib".

## HOW TO RUN THE EXAMPLES

Try this:
```sh
cp waf demos/c/
cd demos/c/
./waf configure build
```

