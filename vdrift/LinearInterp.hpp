#pragma once

#include "PairSorter.hpp"

#include <algorithm>
#include <cassert>
#include <vector>

// Stuntrally's LINEARINTERP class
template <typename T>
class LinearInterp {
public:
//---- BoundMode for out-of-bound extrapolation
	// CONSTANTSLOPE = Use the slope of the closest points to extrapolate
	// CONSTANTVALUE = Return the value of the closest point
	enum BoundMode { CONSTANTSLOPE, CONSTANTVALUE };

	LinearInterp()
		: firstSlope(0.0), lastSlope(0.0), slopesCalculated(false), mode(CONSTANTVALUE),
		  emptyVal(0) { }
	LinearInterp(T ev)
		: firstSlope(0.0), lastSlope(0.0), slopesCalculated(false), mode(CONSTANTVALUE),
		  emptyVal(ev) { }

	void clear() {
		points.clear();
		slopesCalculated = false;
	}

	void addPoint(const T x, const T y) {
		points.push_back(std::pair< T, T >(x, y));
		slopesCalculated = false;

		PairSorterByFirst< T > sorter;
		std::sort(points.begin(), points.end(), sorter);
	}

	T interpolate(T x) const {
		if (points.empty()) { return emptyVal; }

		if (points.size() == 1) { return points[0].second; }

		if (!slopesCalculated) { calculate(); }

		size_t low = 0;
		size_t high = points.size() - 1;
		size_t index;

		// Handle the case where the value is out of bounds.
		if (x > points[high].first) {
			if (mode == CONSTANTSLOPE) {
				return points[high].second +
					   lastSlope * (x - points[high].first);
			} else { return points[high].second; }
		}
		if (x < points[low].first) {
			if (mode == CONSTANTSLOPE) {
				return points[low].second +
					   firstSlope * (x - points[low].first);
			} else { return points[low].second; }
		}

		// Bisect to find the interval that distance is on.
		while ((high - low) > 1) {
			index = (size_t) ((high + low) / 2.0);

			if (points[index].first > x) { high = index; }
			else						 { low  = index; }
		}

		// Make sure that x_high > x_low.
		const T diff = points[high].first - points[low].first;
		assert(diff >= 0.0);

		const T diffY = points[high].second - points[low].second;

		return diffY * (x - points[low].first) / diff + points[low].second;
	}

	void setBoundaryMode(const BoundMode& m) { mode = m; }

private:
	std::vector<std::pair< T, T > > points;
	mutable T firstSlope;
	mutable T lastSlope;
	mutable bool slopesCalculated;
	BoundMode mode;
	T emptyVal; // LinearInterp returns this if no points are stored

	void calculate() const {
		size_t n = points.size();
		assert(n > 1);

		firstSlope = (points[1  ].second - points[0  ].second) /
					 (points[1  ].first  - points[0  ].first );
		lastSlope  = (points[n-1].second - points[n-2].second) /
					 (points[n-1].first  - points[n-2].first );

		slopesCalculated = true;
	}
};
