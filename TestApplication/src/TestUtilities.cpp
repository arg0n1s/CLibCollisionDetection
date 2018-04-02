#include "..\include\TestUtilities.h"

namespace tests {

	namespace utils {

		void compareCoordinates(const Vector3d& correct, const Vector3d& given, const string& testname) {
			BOOST_TEST(correct.x() == given.x(), "\n Error for x-coordinates in: " << testname << ". Given x value was: " << given.x() << ", but should have been: " << correct.x() << ".");
			BOOST_TEST(correct.y() == given.y(), "\n Error for y-coordinates in: " << testname << ". Given y value was: " << given.y() << ", but should have been: " << correct.y() << ".");
			BOOST_TEST(correct.z() == given.z(), "\n Error for z-coordinates in: " << testname << ". Given z value was: " << given.z() << ", but should have been: " << correct.z() << ".");
		}

		void compareOrientation(const Quaternion& correct, const Quaternion& given, const string& testname) {
			BOOST_TEST(correct.x() == given.x(), "\n Error for x-value in: " << testname << ". Given x value was: " << given.x() << ", but should have been: " << correct.x() << ".");
			BOOST_TEST(correct.y() == given.y(), "\n Error for y-value in: " << testname << ". Given y value was: " << given.y() << ", but should have been: " << correct.y() << ".");
			BOOST_TEST(correct.z() == given.z(), "\n Error for z-value in: " << testname << ". Given z value was: " << given.z() << ", but should have been: " << correct.z() << ".");
			BOOST_TEST(correct.w() == given.w(), "\n Error for w-value in: " << testname << ". Given w value was: " << given.w() << ", but should have been: " << correct.w() << ".");
		}

		void checkRequiredIds(const IdSet& givenIDs, const IdSet& requiredIDs, const string& testname) {
			for (auto id : requiredIDs) {
				BOOST_TEST((givenIDs.find(id) != givenIDs.end()) == true, "\n Error did not find required ID in: " << testname << ". Required ID was: " << id << ".");
			}
		}
	}

}