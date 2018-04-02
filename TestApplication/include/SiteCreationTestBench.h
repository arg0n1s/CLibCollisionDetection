#pragma once
#include "..\include\TestBench.h"

namespace tests {
	class SiteCreationTestBench : public TestBench {
	public:
		SiteCreationTestBench(bool showVisualization) : TestBench(showVisualization) {};
		virtual void setup();
		virtual void runAllTests();
	private:
		void siteThroughKarthesianAbsoluteTest_setup();
		void siteThroughKarthHullPointerTest_setup();
		void siteThroughParametrizedAbsoluteTest_setup();
		void siteThroughParamHullPointerTest_setup();

		void siteThroughKarthesianAbsoluteTest();
		void siteThroughKarthHullPointerTest();
		void siteThroughParametrizedAbsoluteTest();
		void siteThroughParamHullPointerTest();
	};
}
