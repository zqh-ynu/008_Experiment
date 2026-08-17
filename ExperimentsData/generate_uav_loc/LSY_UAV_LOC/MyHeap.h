#pragma once
#include <stdio.h> 
#include <stdlib.h>
#include <time.h>
//#include <iostream>
using namespace std;

const int MAX_HEAP_ELEMENTS = 1024;
template <class T>
class MyHeap
{
public:
	int sonNum;//子结点个数，4的时候效率最高 

	//void setNodeSize(int);
	void insertHeap(T);
	T popHeap();
	void swapObj(int index1, int index2);
	int getMaxIndex(int father);
	void shiftDown(int i);
	void recreateMaxHeap();
	void heapSort();

public:
	MyHeap();
	MyHeap(int, int);
	~MyHeap();
	//MyHeap();

//protected:
public:
	T storeHeapNode[MAX_HEAP_ELEMENTS]; // store the two dimensional array into one sequential array
	int numHeapNode = 0;
};

//template<class T>
//inline void MyHeap<T>::setNodeSize(int sizeOfNodes)
//{
//	numHeapNode = sizeOfNodes;
//}

template<class T>
inline void MyHeap<T>::insertHeap(T newNode)
{
	int son = numHeapNode;
	storeHeapNode[numHeapNode] = newNode;
	numHeapNode++;
	if (numHeapNode == 1)
	{
		return;
	}

	int father, maxIndex;

	while (true)
	{
		father = (son - 1) / sonNum;
		maxIndex = getMaxIndex(father);
		if (maxIndex == son)
		{
			swapObj(father, son);
			son = father;
			if (son == 0) break;
		}
		else
		{
			break;
		}
	}
}

template<class T>
inline T MyHeap<T>::popHeap()
{
	T top = storeHeapNode[0];
	//交换堆顶元素与最后一个元素。也就是把最大的元素放到最后 
	swapObj(numHeapNode - 1, 0);
	numHeapNode--; //先n--，然后调整堆 
	shiftDown(0); //堆顶元素被交换掉了。所以要从顶上开始调整堆 
	return top;
}

template<class T>
inline void MyHeap<T>::swapObj(int index1, int index2)
{
	T temp = storeHeapNode[index1];
	storeHeapNode[index1] = storeHeapNode[index2];
	storeHeapNode[index2] = temp;
}

//参数是father的index。根据指定的sonNum个子节点进行计算每个子节点的下标
template<class T>
inline int MyHeap<T>::getMaxIndex(int father)
{
	int maxIndex = father; //默认father是最大值的编号
	int sonIndex = 0;

	//计算father及它的sonNum个子节点之间的最大值的编号 
	for (int i = 0; i < sonNum; i++) //遍历sonNum个子节点 
	{
		sonIndex = father * sonNum + 1 + i; //根据父节点的index，计算子节点的index 
		if (sonIndex < numHeapNode && storeHeapNode[maxIndex] < storeHeapNode[sonIndex])
		{
			maxIndex = sonIndex; //更新最大结点的index 
		}
	}
	return maxIndex; //返回最大值的编号 
}

//向下调整。i是父节点的编号  
template<class T>
inline void MyHeap<T>::shiftDown(int index)
{
	int maxIndex = 0;
	while (true)
	{
		maxIndex = getMaxIndex(index); //找到father、sonNum个儿子中的最大值

		//如果father不是最大的值 (也就是说不符合堆的定义)，需要要调整
		//就是把编号为maxIndex的那个值与father进行交换  
		if (maxIndex != index)
		{
			swapObj(maxIndex, index); //交换  
			index = maxIndex; //更新i的编号，继续下一轮循环 
		}
		else //符合堆的定义时，退出循环 
		{
			break;
		}
	}
}

//建立大顶堆
template<class T>
inline void MyHeap<T>::recreateMaxHeap()
{
	//最后一个有子节点的元素的编号是 (n - 1) / sonNum，从它开始调整
	for (int i = (numHeapNode - 2) / sonNum; i >= 0; i--)
	{
		shiftDown(i);
	}
}

template<class T>
inline void MyHeap<T>::heapSort()
{
	//createMaxHeap(); //建堆 

	int size2 = numHeapNode; //备份numHeapNode 
	while (numHeapNode > 0) //排序
	{
		//交换堆顶元素与最后一个元素。也就是把最大的元素放到最后 
		swapObj(numHeapNode - 1, 0);
		numHeapNode--; //先n--，然后调整堆 
		shiftDown(0); //堆顶元素被交换掉了。所以要从顶上开始调整堆 
	}
	numHeapNode = size2; //恢复numHeapNode
}

template<class T>
inline MyHeap<T>::MyHeap()
{
	numHeapNode = 0;
	sonNum = 2;
}

template<class T>
inline MyHeap<T>::MyHeap(int size, int numOfSon)
{
	numHeapNode = size;
	sonNum = numOfSon;
}

template<class T>
inline MyHeap<T>::~MyHeap()
{
}
