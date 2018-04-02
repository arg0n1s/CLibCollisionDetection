#pragma once

#include "..\include\TestBench.h"

namespace tests {
	class OctTreeTestBench : public TestBench {
	public:
		OctTreeTestBench(bool showVisualization) : TestBench(showVisualization) {};
		virtual void setup();
		virtual void runAllTests();
	private:
		void testOctTreeConstrution_setup();
		void testOctTreeNearestSearch_setup();
		void testOctTreeCollision_setup();

		void testOctTreeConstrution();
		void testOctTreeNearestSearch();
		void testOctTreeCollision();
	};
}