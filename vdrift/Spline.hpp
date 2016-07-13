#pragma once

#include "PairSorter.hpp"

#include <algorithm>
#include <cassert>
#include <vector>

// Based on vdrift/spline.h
template <typename T>
class Spline {
public:
	Spline()
		: firstSlope(0.0), lastSlope(0.0),
		  derivsCalculated(false), slope(0.0) {}

	void clear() {
		points.clear();
		derivsCalculated = false;
		slope = 0.0;
	}

	void addPoint(const T x, const T y) {
		points.push_back(std::pair< T, T >(x, y));
		derivsCalculated = false;

		PairSorterByFirst< T > sorter; // Sort by x-coordinates
		std::sort(points.begin(), points.end(), sorter);
	}

	T interpolate(T x) const {
		if (points.size() == 1) {
			slope = 0.0;
			return points[0].second;
		}

		if (!derivsCalculated)
			calculate();

		size_t low = 0;
		size_t high = points.size() - 1;
		size_t index;

		// Bisect to find the interval that the distance is on.
		while ((high - low) > 1 ) {
			index = size_t ((high + low) / 2.0);
			if (points[index].first > x)
				high = index;
			else
				low = index;
		}

		const T diff = points[high].first - points[low].first;
		assert(diff >= 0.0);

		// Evaluate the coefficients for the cubic spline equation.
		const T a = (points [high].first - x) / diff;
		const T b = 1.0 - a;
		const T sq = diff*diff / 6.0;
		const T a2 = a*a;
		const T b2 = b*b;

		// Find the first derivative.
		slope = (points[high].second - points[low].second) /diff -
				((3.0 * a2) - 1.0) / 6.0 * diff * secondDeriv [low] +
				((3.0 * b2) - 1.0) / 6.0 * diff * secondDeriv [high];

		// Return the interpolated value.
		return a * points[low].second +
			   b * points[high].second +
			   a * (a2 - 1.0) * sq * secondDeriv[low] +
			   b * (b2 - 1.0) * sq * secondDeriv[high];
	}

private:
	void calculate() const {
		size_t n = points.size();
		assert(n > 1);

		T* a = new T[n];
		T* b = new T[n];
		T* c = new T[n];
		T* r = new T[n];

		// Fill in the arrays that represent the tridiagonal matrix.
		// a[0] is not used.
		T diff = points[1].first - points[0].first;
		b[0] = diff / 3.0;
		c[0] = diff / 6.0;
		r[0] = ((points [1].second - points[0].second) / diff) - firstSlope;

		for (size_t i = 1; i < n - 1; i++) {
			T diff1 = points[i+1].first - points[i].first;
			T diff2 = points[i].first - points[i-1].first;

			a [i] = diff2 / 6.0;
			b [i] = (points [i+1].first - points [i-1].first) / 3.0;
			c [i] = diff1 / 6.0;
			r [i] = ((points [i+1].second - points [i].second) / diff1) -
					((points [i].second - points [i-1].second) / diff2);
		}

		diff = points[n-1].first - points[n-2].first;
		a[n-1] = diff / 6.0;
		b[n-1] = diff / 3.0;
		// c[n-1] is not used.
		r[n-1] = lastSlope - ((points [n-1].second - points [n-2].second) / diff);

		// Gauss-Jordan Elimination
		for (size_t i = 1; i < n; i++) {
			// Replace row i with row i - k * row (i-1) such that A_{i,i-1} = 0.0.
			T factor = a[i] / b[i-1];
			// A_{i,i-1} is not used again, so it need not be calculated.
			b[i] -= factor * c[i-1];
			// A_{i,i+1} is unchanged because A_{i-1,i+1} = 0.0.
			r[i] -= factor * r[i-1];
		}

		// Back-substitution

		// Solve for y"[N].
		secondDeriv.resize(n);
		secondDeriv[n-1] = r [n-1] / b [n-1];
		for (int i = n - 2; i >= 0; i--)
		{
			// Use the solution for y"[i+1] to find y"[i].
			secondDeriv[i] = (r[i] - (c[i] * secondDeriv[i+1])) / b[i];
		}

		delete[] a;
		delete[] b;
		delete[] c;
		delete[] r;

		derivsCalculated = true;
	}

	std::vector<std::pair< T, T > > points;
	mutable std::vector< T > secondDeriv;
	T firstSlope;
	T lastSlope;
	mutable bool derivsCalculated;
	mutable T slope;
};
