#include "strategy/fMatrix.h"
#include <iostream>
#include <cstring>
using namespace std;

fMatrix::fMatrix(int n_rows, int n_cols)
{
	this->rows = n_rows;
	this->cols = n_cols;
	this->elem = new Float[rows * cols];
	for (int i = 0; i < rows*cols; i++)this->elem[i] = 0;
}
fMatrix::fMatrix(Float *Array, int n_rows, int n_cols)
{
	this->rows = n_rows;
	this->cols = n_cols;
	this->elem = new Float[n_rows * n_cols];
	memcpy(this->elem, Array, n_rows * n_cols * sizeof(Float));
}
fMatrix::fMatrix(int n_rows, int n_cols, Float *Array)
{
	this->rows = n_rows;
	this->cols = n_cols;
	this->elem = new Float[n_rows * n_cols];
	memcpy(this->elem, Array, n_rows * n_cols * sizeof(Float));
}
fMatrix::fMatrix(const fMatrix &matrixA)//SVD���ΰ�
{
	this->rows = matrixA.rows;
	this->cols = matrixA.cols;
	this->elem = new Float[rows * cols];
	memcpy(this->elem, matrixA.elem, rows * cols * sizeof(Float));
}
fMatrix::~fMatrix()
{

}

fMatrix operator + (const fMatrix &matrixA, const fMatrix &matrixB)
{
	fMatrix Buffer(matrixA.rows, matrixA.cols);
	for (int i = 0; i < matrixA.rows; i++)
	for (int j = 0; j < matrixA.cols; j++)
		Buffer.elem[(i * matrixA.cols) + j] = matrixA.elem[(i * matrixA.cols) + j] + matrixB.elem[(i * matrixA.cols) + j];
	return Buffer;
}

fMatrix operator - (const fMatrix &matrixA)
{
	fMatrix Buffer(matrixA.rows, matrixA.cols);
	for (int i = 0; i < matrixA.rows; i++)
	for (int j = 0; j < matrixA.cols; j++)
		Buffer.elem[(i * matrixA.cols) + j] = matrixA.elem[(i * matrixA.cols) + j] * (-1);
	return Buffer;
}

fMatrix operator - (const fMatrix &matrixA, const fMatrix &matrixB)
{
	fMatrix Buffer(matrixA.rows, matrixA.cols);
	for (int i = 0; i < matrixA.rows; i++)
	for (int j = 0; j < matrixA.cols; j++)
		Buffer.elem[(i * matrixA.cols) + j] = matrixA.elem[(i * matrixA.cols) + j] - matrixB.elem[(i * matrixA.cols) + j];
	return Buffer;
}

fMatrix operator * (const fMatrix &matrixA, Float num)
{
	fMatrix Buffer(matrixA.rows, matrixA.cols);
	for (int i = 0; i < matrixA.rows; i++)
	for (int j = 0; j < matrixA.cols; j++)
		Buffer.elem[(i * matrixA.cols) + j] = matrixA.elem[(i * matrixA.cols) + j] * num;
	return Buffer;
}

fMatrix operator * (Float num, const fMatrix &matrixA)
{
	fMatrix Buffer(matrixA.rows, matrixA.cols);
	for (int i = 0; i < matrixA.rows; i++)
	for (int j = 0; j < matrixA.cols; j++)
		Buffer.elem[(i * matrixA.cols) + j] = matrixA.elem[(i * matrixA.cols) + j] * num;
	return Buffer;
}

fMatrix operator / (const fMatrix &matrixA, Float num)
{
	fMatrix Buffer(matrixA.rows, matrixA.cols);
	for (int i = 0; i < matrixA.rows; i++)
	for (int j = 0; j < matrixA.cols; j++)
		Buffer.elem[(i * matrixA.cols) + j] = matrixA.elem[(i * matrixA.cols) + j] / num;
	return Buffer;
}

fMatrix operator * (const fMatrix &matrixA, const fMatrix &matrixB)//2.3 * 3.2 = 2.2   inner = 3
{
	fMatrix Buffer(matrixA.rows, matrixB.cols);
	for (int i = 0; i < matrixA.rows; i++)
	for (int j = 0; j < matrixB.cols; j++)
	for (int inner = 0; inner < matrixA.cols; inner++)
		Buffer.elem[(i * Buffer.cols) + j] += matrixA.elem[(i * matrixA.cols) + inner] * matrixB.elem[(inner * matrixB.cols) + j];
	return Buffer;
}

fVector operator * (const fMatrix &matrixA, const fVector &vectorA)
{
	fVector Buffer(matrixA.cols);
	Float *vectorAelem = new Float[matrixA.cols];
	Float *Bufferelem = new Float[matrixA.cols];
	for (int i = 0; i < matrixA.cols; i++)Bufferelem[i] = 0;
	vectorA.Getelem(vectorAelem);
	for (int i = 0; i < matrixA.cols; i++)
	for (int inner = 0; inner < matrixA.cols; inner++)
		Bufferelem[i] += matrixA.elem[(i * matrixA.cols) + inner] * vectorAelem[inner];
	Buffer.Setelem(Bufferelem);
	delete[] vectorAelem;
	delete[] Bufferelem;
	return Buffer;
}

fVector operator * (const fVector &vectorA, const fMatrix &matrixA)
{
	fVector Buffer(matrixA.rows);
	Float *vectorAelem = new Float[matrixA.rows];
	Float *Bufferelem = new Float[matrixA.rows];
	for (int i = 0; i < matrixA.cols; i++)Bufferelem[i] = 0;
	vectorA.Getelem(vectorAelem);
	for (int i = 0; i < matrixA.rows; i++)
	for (int inner = 0; inner < matrixA.rows; inner++)
		Bufferelem[i] += vectorAelem[inner] * matrixA.elem[(inner * matrixA.cols) + i];
	Buffer.Setelem(Bufferelem);
	delete[] vectorAelem;
	delete[] Bufferelem;
	return Buffer;
}

fMatrix& operator += (fMatrix &matrixA, const fMatrix &matrixB)
{
	for (int i = 0; i < matrixA.rows; i++)
	for (int j = 0; j < matrixA.cols; j++)
		matrixA.elem[(i * matrixA.cols) + j] = matrixA.elem[(i * matrixA.cols) + j] + matrixB.elem[(i * matrixA.cols) + j];
	return matrixA;
}

fMatrix& operator -= (fMatrix &matrixA, const fMatrix &matrixB)
{
	for (int i = 0; i < matrixA.rows; i++)
	for (int j = 0; j < matrixA.cols; j++)
		matrixA.elem[(i * matrixA.cols) + j] = matrixA.elem[(i * matrixA.cols) + j] - matrixB.elem[(i * matrixA.cols) + j];
	return matrixA;
}

fMatrix& operator *= (fMatrix &matrixA, Float num)
{
	for (int i = 0; i < matrixA.rows; i++)
	for (int j = 0; j < matrixA.cols; j++)
		matrixA.elem[(i * matrixA.cols) + j] = matrixA.elem[(i * matrixA.cols) + j] * num;
	return matrixA;
}

fMatrix& operator /= (fMatrix &matrixA, Float num)
{
	for (int i = 0; i < matrixA.rows; i++)
	for (int j = 0; j < matrixA.cols; j++)
		matrixA.elem[(i * matrixA.cols) + j] = matrixA.elem[(i * matrixA.cols) + j] / num;
	return matrixA;
}

fMatrix& operator *= (fMatrix &matrixA, const fMatrix &matrixB)
{
	fMatrix Buffer(matrixA.rows, matrixB.cols);
	for (int i = 0; i < matrixA.rows; i++)
	for (int j = 0; j < matrixB.cols; j++)
	for (int inner = 0; inner < matrixA.cols; inner++)
		Buffer.elem[(i * matrixA.cols) + j] += matrixA.elem[(i * matrixA.cols) + inner] * matrixB.elem[(inner * matrixA.cols) + j];
	matrixA = Buffer;
	return matrixA;
}

fVector& operator *= (fVector &vectorA, const fMatrix &matrixA)
{
	fVector Buffer(matrixA.cols);
	Float *vectorAelem = new Float[matrixA.cols];
	Float *Bufferelem = new Float[matrixA.cols];
	for (int i = 0; i < matrixA.cols; i++)Bufferelem[i] = 0;
	vectorA.Getelem(vectorAelem);
	for (int i = 0; i < matrixA.rows; i++)
	for (int inner = 0; inner < matrixA.rows; inner++)
		Bufferelem[i] += vectorAelem[inner] * matrixA.elem[(inner * matrixA.cols) + i];
	Buffer.Setelem(Bufferelem);
	delete[] vectorAelem;
	delete[] Bufferelem;
	vectorA = Buffer;
	return vectorA;
}

fMatrix &fMatrix::operator=(const fMatrix &matrixA)
{
	this->rows = matrixA.rows;
	this->cols = matrixA.cols;
	memcpy(this->elem, matrixA.elem, rows * cols * sizeof(Float));
	return *this;
}

fMatrix &fMatrix::operator=(Float s)
{
	for (int i = 0; i < rows; i++)
	for (int j = 0; j < cols; j++)
		elem[(i * cols) + j] = s;
	return *this;
}

fMatrix Transp(const fMatrix &matrixA)
{
	fMatrix buffer(matrixA.cols, matrixA.rows);
	Float tmp;
	for (int i = 0; i < matrixA.rows; i++)
	{
		for (int j = 0; j < matrixA.cols; j++)
		{
			tmp = matrixA.elem[i * matrixA.cols + j];
			buffer.elem[j * buffer.cols + i] = matrixA.elem[i * matrixA.cols + j];
		}
	}
	return buffer;
}

fMatrix AATransp(const fMatrix &matrixA)
{
	fMatrix AATransp = matrixA*Transp(matrixA);
	return AATransp;
}

fMatrix ATranspA(const fMatrix &matrixA)
{
	fMatrix ATranspA = Transp(matrixA)*matrixA;
	return ATranspA;
}

fMatrix Outer(const fVector &vectorA, const fVector &vectorB)
{
	fMatrix buffer(vectorA.Getsize(), vectorB.Getsize());
	Float *vectorAelem = new Float[vectorA.Getsize()];
	Float *vectorBelem = new Float[vectorB.Getsize()];
	vectorA.Getelem(vectorAelem);
	vectorB.Getelem(vectorBelem);
	for (int i = 0; i < buffer.rows; i++)
	{
		for (int j = 0; j < buffer.cols; j++)
		{
			buffer.elem[i * buffer.cols + j] = vectorAelem[i] * vectorBelem[j];
		}
	}
	delete[] vectorAelem;
	delete[] vectorBelem;
	return buffer;
}

fMatrix Identity(int nSize)
{
	fMatrix buffer(nSize, nSize);
	for (int i = 0; i < buffer.rows; i++)
	{
		for (int j = 0; j < buffer.cols; j++)
		{
			if (i == j)
			{
				buffer.elem[i * buffer.cols + j] = 1.0;
			}
			else
			{
				buffer.elem[i * buffer.cols + j] = 0.0;
			}
		}
	}
	return buffer;
}

fMatrix Diag(const fVector &vectorA)
{
	fMatrix buffer(vectorA.Getsize(), vectorA.Getsize());
	Float *vectorAelem = new Float[vectorA.Getsize()];
	vectorA.Getelem(vectorAelem);
	for (int i = 0; i < buffer.rows; i++)
	{
		for (int j = 0; j < buffer.cols; j++)
		{
			if (i == j)
			{
				buffer.elem[i * buffer.cols + j] = vectorAelem[i];
			}
			else
			{
				buffer.elem[i * buffer.cols + j] = 0.0;
			}
		}
	}
	return buffer;
}

fVector Diag(const fMatrix &matrixA)
{
	fVector buffer(matrixA.rows);
	Float *bufferelem = new Float[buffer.Getsize()];
	for (int i = 0; i < matrixA.rows; i++)
	{
		for (int j = 0; j < matrixA.cols; j++)
		{
			if (i == j)
			{
				bufferelem[i] = matrixA.elem[i * matrixA.cols + j];
			}
		}
	}
	buffer.Setelem(bufferelem);
	delete[] bufferelem;
	return buffer;
}

fMatrix Diag(Float x, Float y, Float z)
{
	fMatrix buffer(3, 3);
	Float A[3] = { x, y, z };
	for (int i = 0; i < buffer.rows; i++)
	{
		for (int j = 0; j < buffer.cols; j++)
		{
			if (i == j)
			{
				buffer.elem[i * buffer.cols + j] = A[i];
			}
			else
			{
				buffer.elem[i * buffer.cols + j] = 0.0;
			}
		}
	}
	return buffer;
}

double Determinant(const fMatrix &matrixA)
{
	int i, j, m;
	Float tmp;
	double val = 1.0;
	fMatrix buffer(matrixA.rows, matrixA.cols);
	memcpy(buffer.elem, matrixA.elem, matrixA.cols*matrixA.rows * sizeof(Float));
	for (i = 0, j = 0; (i < matrixA.cols) && (j < matrixA.cols); i++, j++)
	{
		if (buffer.elem[i * buffer.cols + j] == 0)
		{
			for (m = i; buffer.elem[m * buffer.cols + j] == 0; m++);
			if (m != buffer.cols)
			{
				for (int n = j; n < buffer.cols; n++)
				{
					tmp = buffer.elem[i * buffer.cols + n];
					buffer.elem[i * buffer.cols + n] = buffer.elem[m * buffer.cols + n];
					buffer.elem[m * buffer.cols + n] = tmp;
				}
				val *= (-1);
			}
		}
		for (int s = buffer.cols - 1; s > i; s--)
		{
			tmp = buffer.elem[s * buffer.cols + j];
			for (int t = j; t < buffer.cols; t++)
			{
				buffer.elem[s * buffer.cols + t] -= buffer.elem[i * buffer.cols + t] * (tmp / buffer.elem[i * buffer.cols + j]);
			}
		}
	}
	for (i = 0; i < buffer.cols; i++)
	{
		val *= buffer.elem[i * buffer.cols + i];
	}
	return val;
}

double Trace(const fMatrix &matrixA)
{
	Float num = 0.0;
	Float tmp;
	for (int i = 0; i < matrixA.rows; i++)
	{
		for (int j = 0; j < matrixA.cols; j++)
		{
			if (i == j)
			{
				tmp = matrixA.elem[i * matrixA.cols + j];
				num += matrixA.elem[i * matrixA.cols + j];
			}
		}
	}
	return num;
}

double OneNorm(const fMatrix &matrixA)
{
	double newnum, num = 0.0;
	fVector buffer(matrixA.rows);
	Float *bufferelem = new Float[buffer.Getsize()];
	for (int i = 0; i < matrixA.cols; i++)
	{
		newnum = 0.0;
		buffer = matrixA.GetCol(i);
		buffer.Getelem(bufferelem);
		for (int j = 0; j < matrixA.rows; j++)
		{
			newnum += abs(bufferelem[j]);
		}
		if (newnum > num)
		{
			num = newnum;
		}
	}
	delete[] bufferelem;
	return num;
}

double InfNorm(const fMatrix &matrixA)
{
	double newnum, num = 0.0;
	fVector buffer(matrixA.cols);
	Float *bufferelem = new Float[buffer.Getsize()];
	for (int i = 0; i < matrixA.rows; i++)
	{
		newnum = 0.0;
		buffer = matrixA.GetRow(i);
		buffer.Getelem(bufferelem);
		for (int j = 0; j < matrixA.cols; j++)
		{
			newnum += abs(bufferelem[j]);
		}
		if (newnum > num)
		{
			num = newnum;
		}
	}
	delete[] bufferelem;
	return num;
}

fMatrix Inverse(const fMatrix &matrixA)
{
	fMatrix buffer(matrixA);
	buffer.Inv();
	return buffer;
}

fMatrix Cholesky(const fMatrix &matrixA)
{
	int tmp1, tmp2, count = 2;
	Float tmp = 0.0;
	fMatrix buffer(matrixA.rows, matrixA.cols);
	for (int i = 0; i < buffer.cols * buffer.rows; i++)
		buffer.elem[i] = 0.0;

	buffer.elem[0] = sqrt(matrixA.elem[0]);

	for (int i = 1; i < matrixA.rows*matrixA.cols; i++)
	{
		tmp2 = i%matrixA.rows;
		tmp1 = i / matrixA.rows;

		if (tmp2 <= tmp1)
		{
			if (i%matrixA.rows == 0)
			{
				buffer.elem[i] = matrixA.elem[i] / buffer.elem[0];
			}
			else if (i % (matrixA.rows + 1) == 0)
			{
				for (int j = i; j > i - count; j--)
				{
					tmp += pow(buffer.elem[j], 2);
				}
				buffer.elem[i] = sqrt(matrixA.elem[i] - tmp);
				tmp = 0.0;
				count++;
			}
			else
			{
				for (int j = 0; j < tmp2; j++)
				{
					tmp += buffer.elem[tmp1*matrixA.rows + j] * buffer.elem[(tmp1 - 1)*matrixA.rows + j];
				}
				buffer.elem[i] = (matrixA.elem[i] - tmp) / buffer.elem[(tmp1 - 1)*matrixA.rows + tmp2];
				tmp = 0.0;
			}
		}
		else
		{
			buffer.elem[i] = 0;
		}
	}
	return buffer;
}

fVector Mean(const fMatrix &matrixA)
{
	fVector buffer(matrixA.rows);
	Float *vectorAelem = new Float[buffer.Getsize()];
	Float num = 0.0;
	for (int i = 0; i < matrixA.cols; i++)
	{
		num = 0.0;
		for (int j = 0; j < matrixA.rows; j++)
		{
			num += matrixA.elem[j * matrixA.cols + i];
		}
		num = num / matrixA.rows;
		vectorAelem[i] = num;
	}
	buffer.Setelem(vectorAelem);
	delete[] vectorAelem;
	return buffer;
}

fMatrix Cov(const fMatrix &matrixA)
{
	fMatrix Mbuffer(matrixA.rows, matrixA.cols);
	fMatrix buffer(matrixA);
	fVector meanVbuffer(matrixA.rows);
	Float *meanVbufferelem = new Float[meanVbuffer.Getsize()];
	Float tmp = 0.0;
	for (int j = 0; j < matrixA.cols; j++)
	{
		tmp = 0.0;
		for (int i = 0; i < matrixA.rows; i++)
		{
			tmp += matrixA.elem[i * matrixA.cols + j];
		}
		tmp /= meanVbuffer.Getsize();
		meanVbufferelem[j] = tmp;
	}
	for (int j = 0; j < matrixA.rows; j++)
	{
		for (int i = 0; i < matrixA.cols; i++)
		{
			Mbuffer.elem[i*matrixA.cols + j] = meanVbufferelem[j];
		}
	}
	meanVbuffer.Setelem(meanVbufferelem);
	buffer = buffer - Mbuffer;
	Mbuffer = Transp(buffer);
	buffer = Mbuffer * buffer;
	buffer /= (buffer.cols - 1);
	delete[] meanVbufferelem;
	return buffer;
}

fMatrix  Cov(const fVector &vectorA)
{
	fMatrix buffer(vectorA.Getsize(), vectorA.Getsize());
	double MeanValue = Mean(vectorA);
	fVector Vbuffer(vectorA);
	Float *Vbufferelem = new Float[Vbuffer.Getsize()];
	Vbuffer = Vbuffer - MeanValue;
	Vbuffer.Getelem(Vbufferelem);
	for (int i = 0; i < buffer.rows; i++)
	{
		for (int j = 0; j < buffer.cols; j++)
		{
			buffer.elem[i*buffer.cols + j] = Vbufferelem[i] * Vbufferelem[j];
		}
	}
	buffer /= (buffer.cols - 1);
	delete[] Vbufferelem;
	return buffer;
}

void fMatrix::Show() const
{
	for (int i = 0; i < rows; i++)
	{
		for (int j = 0; j < cols; j++)
		{
			printf("%lf\t", elem[(i * cols) + j]);
		}
		printf("\n");
	}
}

fVector fMatrix::GetCol(int col) const
{
	fVector buffer(this->cols);
	Float *bufferelem = new Float[buffer.Getsize()];
	for (int i = 0; i < this->rows; i++)
		bufferelem[i] = this->elem[i * this->cols + col];
	buffer.Setelem(bufferelem);
	delete[] bufferelem;
	return buffer;
}

fVector fMatrix::GetRow(int row) const
{
	fVector buffer(this->rows);
	Float *bufferelem = new Float[buffer.Getsize()];
	for (int i = 0; i < this->cols; i++)
		bufferelem[i] = this->elem[row * this->cols + i];
	buffer.Setelem(bufferelem);
	delete[] bufferelem;
	return buffer;
}

fMatrix &fMatrix::Inv()
{
	Float tmp;
	fMatrix IMatrix(this->rows, this->cols);
	memcpy(IMatrix.elem, this->elem, IMatrix.rows * IMatrix.cols * sizeof(Float));
	for (int i = 0; i < this->rows; i++)
	{
		for (int j = 0; j < this->cols; j++)
		{
			if (i == j)
			{
				this->elem[i * this->cols + j] = 1.0;
			}
			else
			{
				this->elem[i * this->cols + j] = 0.0;
			}
		}

	}
	for (int n = 0; n < IMatrix.rows; n++)
	{
		if (IMatrix.elem[n + n*IMatrix.cols] != 1)
		{
			tmp = IMatrix.elem[n + n*IMatrix.cols];
			for (int i = 0; i < IMatrix.cols; i++)
			{
				IMatrix.elem[n * IMatrix.rows + i] /= tmp;
				this->elem[n * this->rows + i] /= tmp;
			}
		}
		for (int i = 0; i < IMatrix.rows; i++)
		{
			if (i != n)
			{
				if (IMatrix.elem[i * IMatrix.cols + n] != 0)
				{
					tmp = IMatrix.elem[i * IMatrix.cols + n];
					for (int j = 0; j < IMatrix.rows; j++)
					{
						IMatrix.elem[i * IMatrix.cols + j] -= IMatrix.elem[n * IMatrix.cols + j] * tmp;
						this->elem[i * this->cols + j] -= this->elem[n * this->cols + j] * tmp;
					}

				}
			}
		}
	}
	return *this;
}


void fMatrix::SetCol(int col, const fVector & Copyelem)
{
	for (int i = 0; i < this->rows; i++)
		this->elem[this->cols*i + col] = Copyelem.receiveelem(i);
}

void fMatrix::SetRow(int row, const fVector & Copyelem)
{
	for (int i = 0; i < this->cols; i++)
		this->elem[this->cols*row + i] = Copyelem.receiveelem(i);
}

