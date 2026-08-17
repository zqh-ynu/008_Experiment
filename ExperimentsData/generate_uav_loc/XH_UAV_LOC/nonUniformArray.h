#pragma once
#include<stdio.h>
#include<assert.h>
#include<vector>
#include<unordered_set>
using namespace std;

const int MAX_ROWS = 200;
const int MAX_ELEMENTS = 20000;

// this class defines a data structure, called non-uniform array,
// which is a two-dimensional array, but the sizes of different rows
// may vary significantly.
// The reason for defining this data structure is that it is very time-comsuming
// to do the two operations of `new' and 'delete', since a 'new' and and a 'delete'
// operation are need for the adjacent list of each vertex. 
// Therefore, there are O(n) such operations.
// The interal data structure of the non-uniform array in fact is a one-dementional array.
// Only O(1) 'new' and 'delete' operations are needed.
// Three basic methods: 
// (1) void allocateMemory(const vector<int> &sizesOfRows)
//      the vector tells the size of row, which are the maximum numbers of 
//        possible elements that can be stored at each row 
// (2) void push_back(int i, int element)
//     store the element at the end of row i
// (3) int & access(int i, int j)
//      access the (j+1) th element at row (i+1)
template <class Type>
class nonUniformArray {
public:
	void allocateMemory(const vector<int>& sizesOfRows);
	void allocateMemory(int* sizesOfRows, int rows);
	inline int size(); // how many rows
	inline int size(int i); // how many elements in row i
	// store the element at the end of row i
	inline void push_back(int i, Type& element);
	inline void pop_back(int i, int sz); // pop the last sz elements
	inline Type& access(int i, int j); // access the element at row i and column j
	void erase(int i, Type element);  // erase the element from row i
	bool find(int i, Type element); // whether the element is contained in row i
	void printArray();
	void clear(); // clear all  elements
	void clear(int i); // clear row i
public:
	nonUniformArray();
	~nonUniformArray();
	nonUniformArray(const nonUniformArray<Type>& other);
	nonUniformArray& operator=(const nonUniformArray<Type>& other);
protected:
	Type storeArray[MAX_ELEMENTS]; // store the two dimensional array into one sequential array
	int maxSize[MAX_ROWS]; // the max size of each row
	int sizeEachRow[MAX_ROWS]; // how many elements stored at each row
	int baseEachRow[MAX_ROWS]; // the base of each row
	int numRows; // total number of rows
};



template <class Type>
inline void nonUniformArray<Type>::clear(int i) // clear row i
{
	//assert(i >= 0 && i < numRows);
	sizeEachRow[i] = 0;
}

template <class Type>
inline void nonUniformArray<Type>::pop_back(int i, int sz)
{
	assert(i >= 0 && i < numRows);
	assert(sz >= 0 && sz <= sizeEachRow[i]);
	sizeEachRow[i] -= sz;
}

template <class Type>
bool nonUniformArray<Type>::find(int i, Type element)
{
	//assert(i >= 0 && i < numRows);
	int sz = sizeEachRow[i];
	int base = baseEachRow[i];
	for (int j = 0; j < sz; ++j)
		if (storeArray[base + j] == element) return true;
	return false;
}

template <class Type>
void nonUniformArray<Type>::erase(int i, Type element)
{  // erase the element from row i
	assert(i >= 0 && i < numRows);
	int j;
	int base = baseEachRow[i];
	int sz = sizeEachRow[i];
	for (j = 0; j < sz; ++j)
		if (storeArray[base + j] == element) break;
	assert(j != sz); // the element must be found
	for (j; j <= sz - 2; ++j)
		storeArray[base + j] = storeArray[base + (j + 1)];
	--sizeEachRow[i];
}

template <class Type>
inline Type& nonUniformArray<Type>::access(int i, int j)
{ // access the element at row i and column j
	//assert(i >= 0 && i < numRows);
	//assert(j >= 0 && j < sizeEachRow[i]);
	return storeArray[baseEachRow[i] + j];
}

template <class Type>
inline void nonUniformArray<Type>::push_back(int i, Type& element)
{ // store the element at the end of row i
	//assert(i >= 0 && i < numRows);
	//assert(sizeEachRow[i] < maxSize[i]); // there are some storage left
	storeArray[baseEachRow[i] + sizeEachRow[i]] = element; // store the element
	++sizeEachRow[i];
}

template <class Type>
inline int nonUniformArray<Type>::size(int i)
{ // how many elements in row i
	//assert(i >= 0 && i < numRows);
	return sizeEachRow[i];
}

template <class Type>
inline int nonUniformArray<Type>::size() // how many rows
{
	return numRows;
}

template <class Type>
void nonUniformArray<Type>::allocateMemory(const vector<int>& sizesOfRows)
{
	assert(sizesOfRows.size() > 0 &&
		sizesOfRows.size() <= MAX_ROWS);

	int i;
	int numElements = 0;
	for (i = 0; i < sizesOfRows.size(); ++i)
	{
		assert(sizesOfRows[i] >= 0);
		maxSize[i] = sizesOfRows[i];
		numElements += maxSize[i]; // count how many elements are needed
		sizeEachRow[i] = 0;
	}
	assert(numElements <= MAX_ELEMENTS);

	numRows = sizesOfRows.size();

	int base = 0;
	for (i = 0; i < numRows; ++i)
	{
		baseEachRow[i] = base;
		base += maxSize[i];
	}
}

template <class Type>
void nonUniformArray<Type>::allocateMemory(int* sizesOfRows, int rows)
{
	//assert(rows > 0 && rows <= MAX_ROWS);
	int i;
	int numElements = 0;
	for (i = 0; i < rows; ++i)
	{
		//assert(sizesOfRows[i] >= 0);
		maxSize[i] = sizesOfRows[i];
		numElements += maxSize[i]; // count how many elements are needed
		sizeEachRow[i] = 0;
	}
	assert(numElements <= MAX_ELEMENTS);

	numRows = rows;
	int base = 0;
	for (i = 0; i < numRows; ++i)
	{
		baseEachRow[i] = base;
		base += maxSize[i];
	}
}

template <class Type>
nonUniformArray<Type>& nonUniformArray<Type>::operator = (const nonUniformArray<Type>& other)
{
	if (this == &other) return *this;

	numRows = other.numRows;
	int i;
	for (i = 0; i < numRows; ++i)
	{
		maxSize[i] = other.maxSize[i];
		sizeEachRow[i] = other.sizeEachRow[i];
		baseEachRow[i] = other.baseEachRow[i];
	}
	for (i = 0; i < MAX_ELEMENTS; ++i)
		storeArray[i] = other.storeArray[i];

	return *this;
}

template <class Type>
nonUniformArray<Type>::nonUniformArray(const nonUniformArray<Type>& other)
{
	numRows = other.numRows;
	int i;
	for (i = 0; i < numRows; ++i)
	{
		maxSize[i] = other.maxSize[i];
		sizeEachRow[i] = other.sizeEachRow[i];
		baseEachRow[i] = other.baseEachRow[i];
	}
	for (i = 0; i < MAX_ELEMENTS; ++i)
		storeArray[i] = other.storeArray[i];
}

template <class Type>
void nonUniformArray<Type>::printArray()
{
	assert(numRows > 0);
	int i, j;
	for (i = 0; i < numRows; ++i)
	{
		printf("row %d:\t", i);
		for (j = 0; j < sizeEachRow[i]; ++j)
			printf("%d ", storeArray[baseEachRow[i] + j]);
		printf("\n");
	}
}

template <class Type>
nonUniformArray<Type>::nonUniformArray()
{
	numRows = 0;
}

template <class Type>
nonUniformArray<Type>::~nonUniformArray()
{
}

template <class Type>
void nonUniformArray<Type>::clear()
{
	numRows = 0;
}

