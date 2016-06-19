#pragma once

#include <map>

// Based on vdrift/pairsort.h
template <typename T>
class PairSorterByFirst {
public:
	bool operator() (const std::pair<T, T>& p1, const std::pair<T, T>& p2) const {
		return p1.first < p2.first;
	}
};

template <typename T>
class PairSorterBySecond {
public:
	bool operator() (const std::pair<T, T>& p1, const std::pair<T, T>& p2) const {
		return p1.second < p2.second;
	}
};
