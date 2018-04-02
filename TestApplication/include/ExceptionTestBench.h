#pragma once
#include "..\include\TestBench.h"

namespace tests {
	class ExceptionTestBench : public TestBench {
	public:
		virtual void setup();
		virtual void runAllTests();
	};
}