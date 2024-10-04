#include "maths.h"

namespace maths {
	double rad2deg(double radians) {
		return radians * 180e0 / M_PI;
	}
	double deg2rad(double degrees) {
		return degrees * M_PI / 180e0;
	}
}


namespace linearAlgebraLib {

	/*Required class utility functions*/
	Vector3::Vector3() : _vector3({ 0,0,0 }) {}
	Vector3::Vector3(double x, double y, double z) : _vector3({ x, y, z }) {}
	Vector3::Vector3(std::array<double, 3> &xyz): _vector3(xyz) {} 

	void Vector3::set(double x, double y, double z) {
		_vector3 = { x,y,z };
	}

	void Vector3::set(std::array<double, 3> &xyz){
		if (xyz.size() == 3)
			_vector3 = xyz;
	}



	const double& Vector3::operator[](unsigned index) const {
		return _vector3[index];
	}
	double& Vector3::operator[](unsigned index) {
		return _vector3[index];
	}

	std::ostream& operator<<(std::ostream& out, const Vector3& vector3) {
		out << "( ";
		for (unsigned index = 0; index < 3; ++index)
			out << vector3[index] << " ";
		out << ")";
		return out;
	}

	/*Mathematical utility functions*/
	Vector3 Vector3::transpose() const {
		//we create a new Vector3 to return, since we dont want our initial Vector3 to be transposed for ever 
		//(e.g. if i do r0.transpose + r0, i dont want r0 to be transposed)  
		Vector3 transposedVector3;
		transposedVector3._vector3 = _vector3;
		transposedVector3._isRowVector = _isRowVector == true ? false : true;
		return transposedVector3;
	}

	double Vector3::Norm() const {
		double L2Norm = 0.0;
		for (int i = 0; i < 3; ++i)
			L2Norm += pow(_vector3[i], 2);
		L2Norm = std::sqrt(L2Norm);
		return L2Norm;
	}

	Vector3 Vector3::operator+(const Vector3& other) {
		Vector3 resultVector3;
		for (unsigned i = 0; i < _vector3.size(); ++i)
			resultVector3._vector3[i] = _vector3[i] + other._vector3[i];
		return resultVector3;
	}

	Vector3 Vector3::operator-(const Vector3& other) {
		Vector3 resultVector3;
		for (unsigned i = 0; i < _vector3.size(); ++i)
			resultVector3._vector3[i] = _vector3[i] - other._vector3[i];
		return resultVector3;
	}

	double Vector3::operator*(const Vector3& other) {
		double sum = 0.0;
		for (unsigned i = 0; i < _vector3.size(); ++i)
			sum += _vector3[i] * other._vector3[i];
		return sum;
	}

	Vector3 Vector3::operator*(const double& scaleFactor) {
		Vector3 vect = (*this);
		for (unsigned index = 0; index < _vector3.size(); ++index)
			vect[index] *= scaleFactor;
		return vect;
	}

	Vector3 operator*(const double& scaleFactor, Vector3 Vector3) {
		for (unsigned index = 0; index < 3; ++index)
			Vector3[index] *= scaleFactor;
		return Vector3;
	}

	Vector3 Vector3::project(Vector3 vector_to_project_on) {
		double scalar = (*this) * vector_to_project_on / pow(vector_to_project_on.Norm(), 2);
		return scalar * vector_to_project_on;
	}

	Vector3 Vector3::eulerAngles(Matrix3 axis) const {
		Vector3 u = (*this);
		Vector3 euler_angles;
		euler_angles[0] = acos( u * axis.i() / (u.Norm() * axis.i().Norm()) );
		euler_angles[1] = acos( u * axis.j() / (u.Norm() * axis.j().Norm()) );
		euler_angles[2] = acos( u * axis.k() / (u.Norm() * axis.k().Norm()) );
		return {euler_angles};
	}

	Vector3 Vector3::eulerAngles() const {
		Matrix3 axis;
		axis.setColumns({ 1,0,0 }, { 0,1,0 }, { 0,0,1 });
		return { this->eulerAngles(axis)};
	}

	void Vector3::setAllElements(const double& value) {
		_vector3 = { value, value, value };
	}

	Vector3 Vector3::getDirectionCosines() {
		const double norm = (*this).Norm();
		return { _vector3[0] / norm, _vector3[1] / norm, _vector3[2] / norm };
	}

	std::vector<double> Vector3::getStdVector(){
		return std::vector<double> {_vector3[0], _vector3[1], _vector3[2]};
	}

	std::array<double, 3> Vector3::getStdArray(){
		return std::array<double, 3> {_vector3[0], _vector3[1], _vector3[2]};
	}


	/// <summary>
	/// /////////////////////////////////////////////////////////
	/// /////////////////////////////////////////////////////////
	/// </summary>
	Matrix3::Matrix3() {}
	Matrix3::Matrix3(Vector3 row1, Vector3 row2, Vector3 row3) : _matrix({ row1, row2, row3 }) {}
	void Matrix3::setRows(Vector3 row1, Vector3 row2, Vector3 row3) {
		_matrix = { {row1, row2, row3} };
	}
	void Matrix3::setColumns(Vector3 col1, Vector3 col2, Vector3 col3) {
		for (int i = 0; i < 3; ++i) {
			_matrix[i][0] = col1[i];
			_matrix[i][1] = col2[i];
			_matrix[i][2] = col3[i];
		}
	}
	Matrix3 Matrix3::transpose() const {
		Matrix3 transposedMatrix3;
		for (int i = 0; i < 3; ++i) {
			for (int j = 0; j < 3; ++j) {
				transposedMatrix3[i][j] = (*this)[j][i];
			}
		}
		//transposedMatrix3._is_column_matrix = _is_column_matrix == true ? false : true;
		return transposedMatrix3;
	}
	void Matrix3::transposeSelf() {
		(*this) = (*this).transpose();
	}

	Vector3 Matrix3::_returnVector(int number) {
		Vector3 vect;
		if (_is_column_matrix) {
			for (int row = 0; row < 3; ++row)
				vect[row] = _matrix[row][number];
		}
		else {
			vect = _matrix[number];
		}
		return vect;
	}
	Vector3 Matrix3::i() {
		return _returnVector(0);
	}
	Vector3 Matrix3::j() {
		return _returnVector(1);
	}
	Vector3 Matrix3::k() {
		return _returnVector(2);
	}

	const Vector3& Matrix3::operator[](int row) const {
		return _matrix[row];
	}
	Vector3& Matrix3::operator[](int row) {
		return _matrix[row];
	}

	std::ostream& operator<<(std::ostream& out, const Matrix3& matrix3) {
		for (int index = 0; index < 3; ++index) {
			out << " " << matrix3[index] << std::endl;
		}
		return out;
	}

	void Matrix3::setAllElements(const double& value) {
		for (int i = 0; i < 3; ++i) {
			for (int j = 0; j < 3; ++j) {
				_matrix[i][j] = value;
			}
		}
	}

	Matrix3 Matrix3::DCM(const Vector3& euler_angles) {
		//DCM is the Orientation of earth axis in respect to the body axis
		//DCM.transpose() is the Orientation of bodyaxis in respect to earth axis
		//returns the rotation matrix from earth to body frame
		Matrix3 DirectionCosineMatrix;
		const double phi = euler_angles[0];
		const double theta = euler_angles[1];
		const double psi = euler_angles[2];
		Vector3 i = { cos(psi) * cos(theta), cos(psi) * sin(phi)  * sin(theta) - cos(phi)*sin(psi), sin(phi) * sin(psi) + cos(phi) * cos(psi) * sin(theta) };
		Vector3 j = { cos(theta) * sin(psi), cos(phi) * cos(psi) + sin(phi) * sin(psi) * sin(theta), cos(phi) * sin(psi) * sin(theta) - cos(psi) * sin(phi) };
		Vector3 k = { -sin(theta) , cos(theta) * sin(phi) , cos(phi) * cos(theta) };
		DirectionCosineMatrix.setColumns(i, j, k);
		return DirectionCosineMatrix;
	}

	bool Matrix3::isColumnMatrix() {
		if (_is_column_matrix)
			return true;
		return false;
	}

	Vector3 Matrix3::operator*(const Vector3& rhs) {
		Vector3 vect;
		if (rhs._isRowVector) {
			std::cerr << "cannot multiply Matrix3 with row vector3\n";
			return { 0, 0, 0 };
		}
		if (!_is_column_matrix) {
			std::cerr << "cannot multiply by left with row matrix3\n";
			return { 0,0,0 };
		}


		for (int row = 0; row < 3; ++row) {
			for (int col = 0; col < 3; ++col) {
				vect[row] += _matrix[row][col] * rhs[col];
			}
		}
		return vect;
	}

	Matrix3 Matrix3::operator*(const double& scaleFactor) {
		Matrix3 mat;
		for (int i = 0; i < 3; ++i) {
			for (int j = 0; j < 3; ++j) {
				mat[i][j] = _matrix[i][j] * scaleFactor;
			}
		}
		return mat;
	}

	Matrix3 operator*(const double& scaleFactor, Matrix3 matrix) {
		Matrix3 mat;
		for (int i = 0; i < 3; ++i) {
			for (int j = 0; j < 3; ++j) {
				mat[i][j] = matrix[i][j] * scaleFactor;
			}
		}
		return mat;

	}

	Base::Base(Vector3 euler_angles) {
		//_matrix = { { }, { }, { }}
	}


	std::vector<std::vector<double>> Matrix3::getStdVectorVector(){
		std::vector<std::vector<double>> tensor_ret(3, std::vector<double> (3,0));
		for (int i=0; i < 3; ++i){
			for (int j=0; j<3;++j){
				tensor_ret[i][j] = _matrix[i][j];
			}
		}

		return tensor_ret;
	}

	

} 