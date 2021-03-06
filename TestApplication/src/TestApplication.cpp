#pragma once
#define BOOST_TEST_MODULE TestApplication TestSuites
#include "..\include\TestUtilities.h"
#include "..\include\ExceptionTestBench.h"
#include "..\include\SiteCreationTestBench.h"
#include "..\include\TransformationTestBench.h"
#include "..\include\OctTreeTestBench.h"

namespace tests {

	void testExceptions() {

		ExceptionTestBench exceptionBench;
		exceptionBench.setup();
		exceptionBench.runAllTests();
	}

	void testAddingSites(bool showVisualization) {
		
		SiteCreationTestBench siteBench(showVisualization);
		siteBench.setup();
		siteBench.runAllTests();
	}

	void testTransformations(bool showVisualization) {

		TransformationTestBench transformBench(showVisualization);
		transformBench.setup();
		transformBench.runAllTests();
	}

	void testOctTree(bool showVisualization) {
		OctTreeTestBench octTreeBench(showVisualization);
		octTreeBench.setup();
		octTreeBench.runAllTests();
	}
}

BOOST_AUTO_TEST_SUITE(Simulation_Objects_test_suite)

BOOST_AUTO_TEST_CASE(Test_Exceptions, *boost::unit_test::tolerance(tests::utils::EPS))
{
	BOOST_CHECK_NO_THROW(tests::testExceptions());
}

BOOST_AUTO_TEST_CASE(Test_Adding_Sites, * boost::unit_test::tolerance(tests::utils::EPS))
{
	BOOST_CHECK_NO_THROW(tests::testAddingSites(false));
}

BOOST_AUTO_TEST_CASE(Test_Transformations, *boost::unit_test::tolerance(tests::utils::EPS))
{
	BOOST_CHECK_NO_THROW(tests::testTransformations(false));
}

BOOST_AUTO_TEST_CASE(Test_OctTeeLibrary, *boost::unit_test::tolerance(tests::utils::EPS))
{
	BOOST_CHECK_NO_THROW(tests::testOctTree(false));
}

BOOST_AUTO_TEST_SUITE_END()