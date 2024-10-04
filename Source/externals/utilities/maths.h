#pragma once
#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>
#include <vector>
#include <array>
namespace maths {
	
	double rad2deg(double radians);
	double deg2rad(double degrees);
	
	const double pi = 4e0 * atan(1e0);

}


namespace linearAlgebraLib {

	class Matrix3;

	class Vector3 {
	public:
		using Vector3Type = std::array<double, 3>;

	public:
		Vector3();
		Vector3(double x, double y, double z);
		Vector3(std::array<double, 3> &xyz);
		// double begin(){ //this is so we can use loop like this : for(double& : vector3_object)
		// 	return _vector3[0];
		// };
		// double end(){//this is so we can use loop like this : for(double& : vector3_object)
		// 	return _vector3[2];
		// }
		void set(double x, double y, double z);
		void set(std::array<double, 3> &xyz);

		Vector3 transpose() const;
		double Norm() const;

		const double& operator[](unsigned index) const; //???
		double& operator[](unsigned index);				//??? why do i need both of those and not only the 2nd?

		Vector3 operator+(const Vector3& other);
		Vector3 operator-(const Vector3& other);

		Vector3 operator*(const double& scaleFactor); //Vector3 * scalar (we only pass the scalar)
		friend Vector3 operator*(const double& scaleFactor, Vector3 Vector3); //scalar * Vector3
		//think about it like that 
		//if i had a const double class, i would have a Vector3 operator*(Vector3 Vector3)
		//but if i want to implement the operator in the Vector3 class, then i do it as a friend
		//i could do it for matrix Vector3 multiplication as well, but i will do it at matrix file
		//(I couldnt actually do the above without pointers because the one class would depend on the other and vice versa)
		double operator*(const Vector3& other);

		friend std::ostream& operator<<(std::ostream& out, const Vector3& Vector3); //

		Vector3 project(Vector3 vector_to_project_on);
		Vector3 rotate(Vector3 euler_angles) const;
		Vector3 eulerAngles(Matrix3 axis) const;
		Vector3 eulerAngles() const;

		void setAllElements(const double& value);
		Vector3 getDirectionCosines();

		std::vector<double> getStdVector();
		std::array<double, 3> getStdArray();

	private:
		Vector3Type _vector3;
		bool _isRowVector = false;

		friend class Matrix3;

	};

	class Matrix3 {
	public:
		Matrix3();
		Matrix3(Vector3 col1, Vector3 col2, Vector3 col3);
		void setRows(Vector3 row1, Vector3 row2, Vector3 row3);
		void setColumns(Vector3 col1, Vector3 col2, Vector3 col3);


		Matrix3 transpose() const;
		void transposeSelf();
		Vector3 i();
		Vector3 j();
		Vector3 k();


		const Vector3& operator[](int row) const; //???
		Vector3& operator[](int row);
		Vector3 operator*(const Vector3& rhs); //matrix * vect
		Matrix3 operator*(const double& scaleFactor); //Matrix3 * scalar (we only pass the scalar)
		friend Matrix3 operator*(const double& scaleFactor, Matrix3 matrix); //scalar * Matrix3
		friend std::ostream& operator<<(std::ostream& out, const Matrix3& Matrix3);
		
		void setAllElements(const double& value);

		static Matrix3 DCM(const Vector3& euler_angles); //returns the rotation matrix from earth to body frame


		bool isColumnMatrix();

		std::vector<std::vector<double>> getStdVectorVector();
		
	protected:
		Vector3 _returnVector(int number);
		bool _is_column_matrix = true; //const for now
		std::array<Vector3, 3> _matrix;

		friend class Vector3;

	};


	class Base: public Matrix3 {
		Base(Vector3 euler_angles);


	};


	

} // namespace linearAlgebraLib