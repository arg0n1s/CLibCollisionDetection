#pragma once

#include "..\include\TestBench.h"

namespace tests {
	class TransformationTestBench : public TestBench {
	public:
		TransformationTestBench(bool showVisualization) : TestBench(showVisualization) {};
		virtual void setup();
		virtual void runAllTests();
	private:
		void testAgentTransformations_setup();
		void testAgentClusterTransformations_setup();

		void testAgentTransformations();
		void testAgentClusterTransformations();
	};
}
