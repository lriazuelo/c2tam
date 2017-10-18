/*                       
	Copyright (C) 2005 Tom Drummond

	This library is free software; you can redistribute it and/or
	modify it under the terms of the GNU Lesser General Public
	License as published by the Free Software Foundation; either
	version 2.1 of the License, or (at your option) any later version.

	This library is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
	Lesser General Public License for more details.

	You should have received a copy of the GNU Lesser General Public
	License along with this library; if not, write to the Free Software
	Foundation, Inc.
     51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
    51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/
//-*- c++ -*-
// A WLS class using Cholesky decomposition and sparse JtJ
// Also stores the sum squared error and can compute the residual

#ifndef __WLS_CHOLESKY_H
#define __WLS_CHOLESKY_H

#include <TooN/TooN.h>
#include <TooN/helpers.h>
#include <cassert>
#include <cmath>

namespace TooN {

/// Performs weighted least squares using Cholesky decomposition and sparse JtJ.
/// Much faster (but less robust) than the standard WLS.
/// Also stores the sum squares error and can compute the residual.
/// @param The number of parameters in the system
/// @ingroup gEquations
template <int Size = -1>
class WLSCholesky {
public:
  /// Default constructor
  WLSCholesky(){clear();}
  /// Construct using a given regularisation prior
  WLSCholesky(double prior){clear(prior);}
  /// Copy constructor
  /// @param w The decomposition object to copy
  WLSCholesky(const WLSCholesky &w) {
    my_C_inv=w.my_C_inv;
    my_err=w.my_err;
    my_extra=w.my_extra;
    my_vector=w.my_vector;
  }

  /// Clear all the measurements and apply a constant regularisation term. 
  /// Equates to a prior that says all the parameters are zero with \f$\sigma^2 = \frac{1}{\text{val}}\f$.
  /// @param prior The strength of the prior
  void clear(double prior=0){
    Identity(my_C_inv,prior);
    for(int i=0; i<Size; i++){
      my_vector[i]=0;
    }
    my_err=0;
    my_extra=0;
  }

  /// Applies a constant regularisation term. 
  /// Equates to a prior that says all the parameters are zero with \f$\sigma^2 = \frac{1}{\text{val}}\f$.
  /// @param val The strength of the prior
  void add_prior(double val){
    for(int i=0; i<Size; i++){
      my_C_inv(i,i)+=val;
    }
  }
  
  /// Applies a constant regularisation term about a non-zero postion
  /// Equates to a prior that says the parameters are equal to pos with \f$\sigma^2 = \frac{1}{\text{val}}\f$.
  /// @param val The strength of the prior
  /// @param pos The position
  template<class Accessor>
  void add_prior(double val, const FixedVector<Size,Accessor>& pos){
    for(int i=0; i<Size; i++){
      my_C_inv(i,i)+=val;
    }
    my_vector+=pos*val;
    my_err+=val*(pos*pos);
  }

	/// Applies a regularisation term with a different strength for each parameter value. 
	/// Equates to a prior that says all the parameters are zero with \f$\sigma_i^2 = \frac{1}{\text{v}_i}\f$.
	/// @param v The vector of priors
  template<class Accessor>
  void add_prior(const FixedVector<Size,Accessor>& v){
    for(int i=0; i<Size; i++){
      my_C_inv(i,i)+=v[i];
    }
  }

	/// Applies a whole-matrix regularisation term. 
	/// This is the same as adding the \f$m\f$ to the inverse covariance matrix.
	/// @param m The inverse covariance matrix to add
  template<class Accessor>
  void add_prior(const FixedMatrix<Size,Size,Accessor>& m){
    my_C_inv+=m;
  }

	/// Add a single measurement 
	/// @param m The value of the measurement
	/// @param J The Jacobian for the measurement \f$\frac{\partial\text{m}}{\partial\text{param}_i}\f$
	/// @param weight The inverse variance of the measurement (default = 1)
  template<class Accessor>
  inline void add_df(double m, const FixedVector<Size,Accessor>& J, double weight = 1) {
    Vector<Size> Jw = J*weight;
    for(int i=0; i<Size; i++){
      for(int j=i; j<Size; j++){
	my_C_inv[i][j]+=J[i]*Jw[j];
      }
      my_vector[i]+=Jw[i]*m;
    }
    my_err+=m*weight*m;
  }

	/// Add multiple measurements at once (much more efficiently)
	/// @param N The number of measurements
	/// @param m The measurements to add
	/// @param J The Jacobian matrix \f$\frac{\partial\text{m}_i}{\partial\text{param}_j}\f$
	/// @param invcov The inverse covariance of the measurement values
  template<int N, class Accessor1, class Accessor2, class Accessor3>
  inline void add_df(const FixedVector<N,Accessor1>& m,
		     const FixedMatrix<Size,N,Accessor2>& J,
		     const FixedMatrix<N,N,Accessor3>& invcov){
    my_C_inv += J * invcov * J.T();  // FIXME make me faster!
    Vector<N> temp(invcov*m);
    my_vector += J * temp;
    my_err += m*temp;
  }

  
  void compute(){
    // Homegrown Cholesky
    Matrix<Size> L;
    for(int i=0;i<Size;i++) {
      double a=my_C_inv[i][i];
      for(int k=0;k<i;k++) a-=L[k][i]*L[k][i];
      L[i][i]=sqrt(a);
      for(int j=i;j<Size;j++) {
	a=my_C_inv[i][j];
	for(int k=0;k<i;k++) a-=L[k][j]*L[k][i];
	L[i][j]=a/L[i][i];
      }
    }
    Vector<Size> y;
    for(int i=0;i<Size;i++) {
      double a=my_vector[i];
      for(int j=0;j<i;j++) a-=L[j][i]*y[j];
      y[i]=a/L[i][i];
    }
    for(int i=Size-1;i>-1;i--) {
      double a=y[i];
      for(int j=i+1;j<Size;j++) a-=L[i][j]*my_mu[j];
      my_mu[i]=a/L[i][i];
    } 
  }

  /// Combine measurements from two WLS systems
  /// @param meas The measurements to combine with
  void operator += (const WLSCholesky& meas){
    my_vector+=meas.my_vector;
    my_C_inv+=meas.my_C_inv;
    my_err+=meas.my_err;
    my_extra+=meas.my_extra;
  }

  /// Copy measurements from another WLS system
  /// @param w The measurements to copy
  void operator = (const WLSCholesky &w) {
    my_C_inv=w.my_C_inv;
    my_err=w.my_err;
    my_vector=w.my_vector;
    my_extra=w.my_extra;
  }

  /// Returns the inverse covariance matrix
  Matrix<Size,Size,RowMajor>& get_C_inv() {return my_C_inv;}
  /// Returns the inverse covariance matrix
  const Matrix<Size,Size,RowMajor>& get_C_inv() const {return my_C_inv;}
  Vector<Size>& get_mu(){return my_mu;}
  const Vector<Size>& get_mu() const {return my_mu;}
  Vector<Size>& get_vector(){return my_vector;}
  const Vector<Size>& get_vector() const {return my_vector;}
  double get_residual(){
    Vector<Size> temp;
    Zero(temp);
    for(int ii=0;ii<Size;ii++) {
      temp[ii]+=my_C_inv[ii][ii]*my_mu[ii];
      for(int jj=ii+1;jj<Size;jj++) {
	temp[ii]+=my_C_inv[ii][jj]*my_mu[jj];
	temp[jj]+=my_C_inv[ii][jj]*my_mu[ii];
      }
    }
    return my_err-my_mu*temp;
  }

  inline void add_extra(double e) {my_extra+=e;}
  inline double get_extra() {return my_extra;}

private:
  Vector<Size> my_mu;
  Matrix<Size,Size,RowMajor> my_C_inv;
  Vector<Size> my_vector;
  double my_err;    // error before optimisation
  double my_extra;  // extra residual error 
};

/// Performs weighted least squares using Cholesky decomposition and sparse JtJ.
/// This is the specialisation for dynamically sized matrizes.
/// Much faster (but less robust) than the standard WLS.
/// Also stores the sum squares error and can compute the residual.
/// @param The number of parameters in the system
/// @ingroup gEquations
template <>
class WLSCholesky<-1> {
public:
  /// Default constructor
  WLSCholesky(){clear();}
  /// Construct using a given regularisation prior
  WLSCholesky(double prior){clear(prior);}
  /// Construct with a given size and optional regularisation prior
  WLSCholesky(int Size, double prior = 0.0){
    resize(Size);
    clear(prior);
  }
  /// Copy constructor
  /// @param w The decomposition object to copy
  WLSCholesky(const WLSCholesky &w) {
    resize(w.size());
    my_C_inv=w.my_C_inv;
    my_err=w.my_err;
    my_extra=w.my_extra;
    my_vector=w.my_vector;
  }

  /// sets the size of the WLSCholesky object equalling the number of parameters to estimate
  void resize(int N){
    my_C_inv.resize(N,N);
    my_vector.resize(N);
    my_mu.resize(N);
  }

  /// returns the size of the object
  int size(void) const throw() {
    return my_vector.size();
  }

  /// Clear all the measurements and apply a constant regularisation term.
  /// Equates to a prior that says all the parameters are zero with \f$\sigma^2 = \frac{1}{\text{val}}\f$.
  /// @param prior The strength of the prior
  void clear(double prior=0){
    Identity(my_C_inv,prior);
    for(int i=0; i<size(); i++){
      my_vector[i]=0;
    }
    my_err=0;
    my_extra=0;
  }

  /// Applies a constant regularisation term.
  /// Equates to a prior that says all the parameters are zero with \f$\sigma^2 = \frac{1}{\text{val}}\f$.
  /// @param val The strength of the prior
  void add_prior(double val){
    for(int i=0; i<size(); i++){
      my_C_inv(i,i)+=val;
    }
  }

  /// Applies a constant regularisation term about a non-zero postion
  /// Equates to a prior that says the parameters are equal to pos with \f$\sigma^2 = \frac{1}{\text{val}}\f$.
  /// @param val The strength of the prior
  /// @param pos The position
  template<class Accessor, int N>
  void add_prior(double val, const FixedVector<N,Accessor>& pos){
    assert(N == size());
    for(int i=0; i<N; i++){
      my_C_inv(i,i)+=val;
    }
    my_vector+=pos*val;
    my_err+=val*(pos*pos);
  }

	/// Applies a regularisation term with a different strength for each parameter value.
	/// Equates to a prior that says all the parameters are zero with \f$\sigma_i^2 = \frac{1}{\text{v}_i}\f$.
	/// @param v The vector of priors
  template<class Accessor, int N>
  void add_prior(const FixedVector<N,Accessor>& v){
    assert(N == size());
    for(int i=0; i<N; i++){
      my_C_inv(i,i)+=v[i];
    }
  }

	/// Applies a whole-matrix regularisation term.
	/// This is the same as adding the \f$m\f$ to the inverse covariance matrix.
	/// @param m The inverse covariance matrix to add
  template<class Accessor, int N>
  void add_prior(const FixedMatrix<N,N,Accessor>& m){
    assert(N == size());
    my_C_inv+=m;
  }

	/// Add a single measurement
	/// @param m The value of the measurement
	/// @param J The Jacobian for the measurement \f$\frac{\partial\text{m}}{\partial\text{param}_i}\f$
	/// @param weight The inverse variance of the measurement (default = 1)
  template<class Accessor, int N>
  inline void add_df(double m, const FixedVector<N,Accessor>& J, double weight = 1) {
    assert(N == size());
    Vector<N> Jw = J*weight;
    for(int i=0; i<N; i++){
      for(int j=i; j<N; j++){
	my_C_inv[i][j]+=J[i]*Jw[j];
      }
      my_vector[i]+=Jw[i]*m;
    }
    my_err+=m*weight*m;
  }

	/// Add a single measurement
	/// @param m The value of the measurement
	/// @param J The Jacobian for the measurement \f$\frac{\partial\text{m}}{\partial\text{param}_i}\f$
	/// @param weight The inverse variance of the measurement (default = 1)
  template<class Accessor>
  inline void add_df(double m, const DynamicVector<Accessor>& J, double weight = 1) {
    assert(J.size() == size());
    const int Size =size();
    Vector<-1> Jw = J*weight;
    for(int i=0; i<Size; i++){
      for(int j=i; j<Size; j++){
	my_C_inv[i][j]+=J[i]*Jw[j];
      }
      my_vector[i]+=Jw[i]*m;
    }
    my_err+=m*weight*m;
  }

	/// Add multiple measurements at once (much more efficiently)
	/// @param N The number of measurements
	/// @param m The measurements to add
	/// @param J The Jacobian matrix \f$\frac{\partial\text{m}_i}{\partial\text{param}_j}\f$
	/// @param invcov The inverse covariance of the measurement values
  template<int N, class Accessor1, class Accessor2, class Accessor3, int M>
  inline void add_df(const FixedVector<N,Accessor1>& m,
		     const FixedMatrix<M,N,Accessor2>& J,
		     const FixedMatrix<N,N,Accessor3>& invcov){
    assert(M == size());
    my_C_inv += J * invcov * J.T();  // FIXME make me faster!
    Vector<N> temp(invcov*m);
    my_vector += J * temp;
    my_err += m*temp;
  }

	/// Add multiple measurements at once (much more efficiently)
	/// @param N The number of measurements
	/// @param m The measurements to add
	/// @param J The Jacobian matrix \f$\frac{\partial\text{m}_i}{\partial\text{param}_j}\f$
	/// @param invcov The inverse covariance of the measurement values
  template<class Accessor1, class Accessor2, class Accessor3>
  inline void add_df(const DynamicVector<Accessor1>& m,
		     const DynamicMatrix<Accessor2>& J,
		     const DynamicMatrix<Accessor3>& invcov){
    // FIXME add some meaningful asserts here
    my_C_inv += J * invcov * J.T();  // FIXME make me faster!
    Vector<-1> temp(invcov*m);
    my_vector += J * temp;
    my_err += m*temp;
  }

  void compute(){
    const int Size = size();
    // Homegrown Cholesky
    Matrix<-1> L(Size, Size);
    my_mu.resize(Size);
    for(int i=0;i<Size;i++) {
      double a=my_C_inv[i][i];
      for(int k=0;k<i;k++) a-=L[k][i]*L[k][i];
      L[i][i]=sqrt(a);
      for(int j=i;j<Size;j++) {
	a=my_C_inv[i][j];
	for(int k=0;k<i;k++) a-=L[k][j]*L[k][i];
	L[i][j]=a/L[i][i];
      }
    }
    Vector<-1> y(Size);
    for(int i=0;i<Size;i++) {
      double a=my_vector[i];
      for(int j=0;j<i;j++) a-=L[j][i]*y[j];
      y[i]=a/L[i][i];
    }
    for(int i=Size-1;i>-1;i--) {
      double a=y[i];
      for(int j=i+1;j<Size;j++) a-=L[i][j]*my_mu[j];
      my_mu[i]=a/L[i][i];
    }
  }

  /// Combine measurements from two WLS systems
  /// @param meas The measurements to combine with
  void operator += (const WLSCholesky& meas){
    assert(size() == meas.size());
    my_vector+=meas.my_vector;
    my_C_inv+=meas.my_C_inv;
    my_err+=meas.my_err;
    my_extra+=meas.my_extra;
  }

  /// Copy measurements from another WLS system
  /// @param w The measurements to copy
  void operator = (const WLSCholesky &w) {
    resize(w.size());
    my_C_inv=w.my_C_inv;
    my_err=w.my_err;
    my_vector=w.my_vector;
    my_extra=w.my_extra;
  }

  /// Returns the inverse covariance matrix
  Matrix<-1,-1,RowMajor>& get_C_inv() {return my_C_inv;}
  /// Returns the inverse covariance matrix
  const Matrix<-1,-1,RowMajor>& get_C_inv() const {return my_C_inv;}
  Vector<-1>& get_mu(){return my_mu;}
  const Vector<-1>& get_mu() const {return my_mu;}
  Vector<-1>& get_vector(){return my_vector;}
  const Vector<-1>& get_vector() const {return my_vector;}
  double get_residual(){
   const int Size = size();
    Vector<-1> temp(Size);
    Zero(temp);
    for(int ii=0;ii<Size;ii++) {
      temp[ii]+=my_C_inv[ii][ii]*my_mu[ii];
      for(int jj=ii+1;jj<Size;jj++) {
	temp[ii]+=my_C_inv[ii][jj]*my_mu[jj];
	temp[jj]+=my_C_inv[ii][jj]*my_mu[ii];
      }
    }
    return my_err-my_mu*temp;
  }

  inline void add_extra(double e) {my_extra+=e;}
  inline double get_extra() {return my_extra;}

private:
  Vector<-1> my_mu;
  Matrix<-1,-1,RowMajor> my_C_inv;
  Vector<-1> my_vector;
  double my_err;    // error before optimisation
  double my_extra;  // extra residual error
};

}

#endif
