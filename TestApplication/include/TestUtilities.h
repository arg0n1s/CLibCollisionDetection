#pragma once
#include <Eigen\Core>
#include <Eigen\Geometry>
#include <string>
#include <unordered_set>
#include <boost/test/unit_test.hpp>

namespace tests {

	namespace utils {

		using std::string;
		using Eigen::Vector3d;
		using Quaternion = Eigen::Quaternion<double>;
		using IdSet = std::unordered_set<unsigned int>;

		static const double EPS = 0.00000001;

		void compareCoordinates(const Vector3d& correct, const Vector3d& given, const string& testname);

		void compareOrientation(const Quaternion& correct, const Quaternion& given, const string& testname);

		void checkRequiredIds(const IdSet& givenIDs, const IdSet& requiredIDs, const string& testname);
	}

}