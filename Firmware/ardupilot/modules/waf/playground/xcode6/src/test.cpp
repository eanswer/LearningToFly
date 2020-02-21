#include <iostream>
#include "MyLib/TestClass.h"

int main(int argc, char const *argv[])
{
	TestClass a;
	std::cout << a.message();
	return 0;
}