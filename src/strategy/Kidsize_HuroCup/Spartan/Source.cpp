#include "strategy/fVector.h"
#include <iostream>
#include <cstring>
using namespace std;
int fVector::nVecCount=0;

fVector::fVector(int size )
{
	this->elem = new Float[size];
	this->size = size;
	for (int i = 0; i < size; i++)this->elem[i] = 0;
	//printf("build nVecCount = %d\n", ++this->nVecCount);
}
fVector::fVector(const fVector &VectorA)
{
	this->size = VectorA.size;
	this->elem = new Float[VectorA.size];
	memcpy(this->elem, VectorA.elem, VectorA.size * sizeof(Float));
	//printf("build nVecCount = %d\n", ++this->nVecCount);
}

fVector::fVector(const Float *x, int n)
{
	this->size = n;
	this->elem = new Float[n];
	memcpy(this->elem, x, n * sizeof(Float));//X bity * n elem //sizeof(this->elem) is show all space (not 1 Float space)
	//printf("build nVecCount = %d\n", ++this->nVecCount);
}

fVector::fVector(int n, const Float *x)
{
	this->size = n;
	this->elem = new Float[n];
	memcpy(this->elem, x, n * sizeof(Float));
	//printf("build nVecCount = %d\n", ++this->nVecCount);
}

fVector::fVector(Float A, Float B)
{
	this->size = 2;
	this->elem = new Float[2];
	this->elem[0] = A;
	this->elem[1] = B;
	//printf("build nVecCount = %d\n", ++this->nVecCount);
}

fVector::fVector(Float A, Float B, Float C)
{
	this->size = 3;
	this->elem = new Float[3];
	this->elem[0] = A;
	this->elem[1] = B;
	this->elem[2] = C;
	//printf("build nVecCount = %d\n", ++this->nVecCount);
}
fVector::~fVector()
{
	delete[] elem;
	//cout << "delete" << endl;
	nVecCount--;


}

void fVector::Show(VecType Type /*= ColVec*/) const
{
	if (Type == ColVec) {
		for (int i = 0; i < size; i++)
			printf("%lf\n", elem[i]);
	}
	else{
		for (int i = 0; i < size; i++) {
			printf("%lf\t", elem[i]);
			if (i == 2)printf("\n");
		}
	}
}
/////////////////////////////////////////////////////////////////////////main//////////////////////////////////////////////////////////////////////////////////////////////////////////
fVector  operator +  (const fVector &object1, const fVector &object2)
{
		fVector ans(object1.size);
		for (int i = 0; i<object1.size; i++)
			ans.elem[i] = object1.elem[i] + object2.elem[i];
		return ans;

}
fVector  operator -  (const fVector &object1, const fVector &object2)
{

		fVector ans(object1.size);
		for (int i = 0; i<object1.size; i++)
			ans.elem[i] = object1.elem[i] - object2.elem[i];
		return ans;

}

fVector  operator -  (const fVector &object1)
{
	fVector ans(object1.size);
	for (int i = 0; i<object1.size; i++)
		ans.elem[i] = -object1.elem[i];
	return ans;
}

fVector  operator -  (const fVector &object1, Float object2)
{
	fVector ans(object1.size);
	for (int i = 0; i<object1.size; i++)
		ans.elem[i] = object1.elem[i] - object2;
	return ans;
}

fVector  operator -  (Float	object1, const fVector &object2)
{
	fVector ans(object2.size);
	for (int i = 0; i<object2.size; i++)
		ans.elem[i] = object1 - object2.elem[i];
	return ans;
}

fVector  operator *  (const fVector & object1, Float object2)
{
	fVector ans(object1.size);
	for (int i = 0; i<ans.size; i++)
		ans.elem[i] = object1.elem[i] * object2;
	return ans;
}
fVector  operator *  (Float object1, const fVector &object2)
{
	fVector ans(object2.size);
	for (int i = 0; i<ans.size; i++)
		ans.elem[i] = object2.elem[i] * object1;
	return ans;
}

fVector  operator /  (const fVector & object1, Float object2)
{
	fVector ans (object1.size);
	for (int i = 0; i<ans.size; i++)
		ans.elem[i] = object1.elem[i] / object2;
	return ans;
}

fVector  operator /  (const fVector & object1, const fVector & object2)
{
	fVector ans(object1.size);
	for (int i = 0; i<ans.size; i++)
		ans.elem[i] = object1.elem[i] / object2.elem[i];
	return ans;
}

double  operator *  (const fVector & object1, const fVector & object2)
{
	double ans=0.0;
	for (int i = 0; i<object1.size; i++)
		ans += object1.elem[i] * object2.elem[i];
	return ans;
}

fVector  operator ^  (const fVector & object1, const fVector & object2)
{
	fVector ans(3);
	ans.elem[0] = object1.elem[1] * object2.elem[2] - object1.elem[2] * object2.elem[1];
	ans.elem[1] = object1.elem[2] * object2.elem[0] - object1.elem[0] * object2.elem[2];
	ans.elem[2] = object1.elem[0] * object2.elem[1] - object1.elem[1] * object2.elem[0];
	return ans;
}

fVector& operator += (fVector &object1, const fVector &object2)
{
	for (int i = 0; i < object1.size; i++)
		object1.elem[i] = object1.elem[i] + object2.elem[i];
	return object1;
}

fVector& operator -= (fVector &object1, const fVector &object2)
{
	for (int i = 0; i < object1.size; i++)
		object1.elem[i] = object1.elem[i] - object2.elem[i];
	return object1;
}

fVector& operator *= (fVector &object1, Float object2)
{
	for (int i = 0; i < object1.size; i++)
		object1.elem[i] = object1.elem[i] * object2;
	return object1;
}

fVector& operator /= (fVector &object1, Float object2)
{
	for (int i = 0; i < object1.size; i++)
		object1.elem[i] = object1.elem[i] / object2;
	return object1;
}

fVector  Min(const fVector &object1, const fVector &object2)
{
	fVector ans(object1.size);
	for (int i = 0; i<object1.size; i++)
		ans.elem[i] = (object1.elem[i] < object2.elem[i]) ? object1.elem[i] : object2.elem[i];
	return ans;
}

fVector  Max(const fVector &object1, const fVector &object2)
{
	fVector ans(object1.size);
	for (int i = 0; i<object1.size; i++)
		ans.elem[i] = (object1.elem[i] < object2.elem[i]) ? object2.elem[i] : object1.elem[i];
	return ans;
}

double   Dist(const fVector &object1, const fVector &object2)
{
	double dist=0;
	for (int i = 0; i < object1.size; i++)
		dist += pow(object1.elem[i] - object2.elem[i], 2);
	return sqrt(dist);
}

fVector  Normalize(const fVector &object1)
{
	fVector tmp(object1.size);
	double distance = 0;
	for (int i = 0; i<object1.size; i++)
		distance += pow(object1.elem[i], 2);
	distance = sqrt(distance);
	for (int i = 0; i<object1.size; i++)
		tmp.elem[i] = object1.elem[i] / distance;
	return tmp;
}

double   OneNorm(const fVector &object)
{
	double distance = 0;
	for (int i = 0; i<object.size; i++)
		distance += abs(object.elem[i]);
	return distance;
}

double   TwoNorm(const fVector &object)
{
	double distance = 0;
	for (int i = 0; i<object.size; i++)
		distance += pow(object.elem[i],2);
	return sqrt(distance);
}

double   TwoNormSqr(const fVector &object)
{
	double distance = 0;
	for (int i = 0; i<object.size; i++)
		distance += pow(object.elem[i], 2);
	return distance;
}

fVector  Sqrt(const fVector &object)
{
	fVector tmp(object.size);
	for (int i = 0; i<object.size; i++)
		tmp.elem[i] = pow(object.elem[i], 2);
	return tmp;
}

double   Mean(const fVector &object)
{
	double distance = 0;
	for (int i = 0; i<object.size; i++)
		distance += object.elem[i];
	return distance / object.size;
}	


double   Var(const fVector &object){
	double distance = 0;
	for (int i = 0; i<object.size; i++)
		distance += object.elem[i];
	distance /= object.size;
	double Variance = 0;
	for (int i = 0; i<object.size; i++)
		Variance += pow(object.elem[i] - distance, 2);
	return Variance / object.size;
}

double   Std(const fVector &object){
	double distance = 0;
	for (int i = 0; i<object.size; i++)
		distance += object.elem[i];
	distance /= object.size;
	double Variance = 0;
	for (int i = 0; i<object.size; i++)
		Variance += pow(object.elem[i] - distance, 2);
	return sqrt(Variance / object.size);
}

double num2double(const fVector &object, int i){
	double ans = 0;
	ans = object.elem[i];
	return ans;
}

void     ShowVector(const fVector &object, VecType Type){
	if (Type == ColVec)
	for (int i = 0; i<object.size; i++)
		cout << object.elem[i] << endl;
	else
	for (int i = 0; i<object.size; i++)
		cout << object.elem[i] << "\t";
}

fVector &fVector::operator=(const fVector &VectorA)
{
	memcpy(this->elem, VectorA.elem, VectorA.size*sizeof(Float));
	return *this;
}

void fVector::operator=(Float A)
{
	for (int i = 0; i < this->size; i++)
		this->elem[i] = A;
}

void fVector::SetSize(int n)
{
	this->size = n;
}

fVector &fVector::Swap(int i, int j)
{
	Float tmp;
	tmp = this->elem[i];
	this->elem[i] = this->elem[j];
	this->elem[j] = tmp;
	return *this;
}

void fVector::Getelem(Float *elem) const
{
	memcpy(elem, this->elem, this->size * sizeof(Float));
}

void fVector::Setelem(Float *elem) const
{
	for (int i = 0; i < size; i++)
		this->elem[i] = elem[i];
}

Float fVector::receiveelem(int i) const
{
	double temp;
	temp = this->elem[i];
	return temp;
}

int fVector::Getsize() const
{
	return this->size;
}







