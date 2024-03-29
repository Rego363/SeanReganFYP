#pragma once

// @generated by aten/src/ATen/gen.py

#include <ATen/CPUTypeDefault.h>
#include <ATen/Context.h>
#include <ATen/CheckGenerator.h>

#include <ATen/DeviceGuard.h>
#include <ATen/cuda/ATenCUDAGeneral.h>
#include <ATen/cuda/CUDADevice.h>
#include <ATen/cuda/CUDATypeDefault.h>

#ifdef _MSC_VER
#ifdef Type
#undef Type
#endif
#endif

namespace at {

struct SparseCUDAByteType final : public CUDATypeDefault {
  explicit SparseCUDAByteType();
  virtual ScalarType scalarType() const override;
  virtual caffe2::TypeMeta typeMeta() const override;
  virtual Backend backend() const override;
  virtual const char * toString() const override;
  virtual TypeID ID() const override;

  // example
  // virtual Tensor * add(Tensor & a, Tensor & b) override;
  Tensor empty(IntArrayRef size, const TensorOptions & options) const override;
  Tensor & log1p_(Tensor & self) const override;
  Tensor & log1p_out(Tensor & out, const Tensor & self) const override;
  Tensor narrow_copy(const Tensor & self, int64_t dim, int64_t start, int64_t length) const override;
  Tensor & _sparse_add_out(Tensor & out, const Tensor & self, const Tensor & other, Scalar alpha) const override;
  Tensor & _sparse_div_zerodim_out(Tensor & out, const Tensor & self, const Tensor & other) const override;
  Tensor & _sparse_div_scalar_out(Tensor & out, const Tensor & self, Scalar other) const override;
  Tensor & _sparse_mul_out(Tensor & out, const Tensor & self, const Tensor & other) const override;
  Tensor & _sparse_mul_zerodim_out(Tensor & out, const Tensor & self, const Tensor & other) const override;
  Tensor & _sparse_mul_scalar_out(Tensor & out, const Tensor & self, Scalar other) const override;
  Tensor & sspaddmm_out(Tensor & out, const Tensor & self, const Tensor & mat1, const Tensor & mat2, Scalar beta, Scalar alpha) const override;
  Tensor native_norm(const Tensor & self, Scalar p) const override;
  Tensor _sparse_sum_backward(const Tensor & grad, const Tensor & self, IntArrayRef dim) const override;
  Tensor native_clone(const Tensor & self) const override;
  Tensor & native_resize_as_(Tensor & self, const Tensor & the_template) const override;
  Tensor & native_pow_out(Tensor & out, const Tensor & self, Scalar exponent) const override;
  Tensor native_pow(const Tensor & self, Scalar exponent) const override;
  Tensor & native_zero_(Tensor & self) const override;
  Tensor _sparse_coo_tensor_with_dims(int64_t sparse_dim, int64_t dense_dim, IntArrayRef size, const TensorOptions & options) const override;
  Tensor _sparse_coo_tensor_with_dims_and_tensors(int64_t sparse_dim, int64_t dense_dim, IntArrayRef size, const Tensor & indices, const Tensor & values, const TensorOptions & options) const override;
  Tensor & sparse_resize_(Tensor & self, IntArrayRef size, int64_t sparse_dim, int64_t dense_dim) const override;
  Tensor & sparse_resize_and_clear_(Tensor & self, IntArrayRef size, int64_t sparse_dim, int64_t dense_dim) const override;
  Tensor to_dense(const Tensor & self) const override;
  int64_t sparse_dim(const Tensor & self) const override;
  int64_t dense_dim(const Tensor & self) const override;
  int64_t _nnz(const Tensor & self) const override;
  Tensor coalesce(const Tensor & self) const override;
  bool is_coalesced(const Tensor & self) const override;
  Tensor _indices(const Tensor & self) const override;
  Tensor _values(const Tensor & self) const override;
  Tensor & _coalesced_(Tensor & self, bool coalesced) const override;
  Tensor indices(const Tensor & self) const override;
  Tensor values(const Tensor & self) const override;
  Tensor & hspmm_out(Tensor & out, const Tensor & mat1, const Tensor & mat2) const override;
  Tensor hspmm(const Tensor & mat1, const Tensor & mat2) const override;
  Tensor & copy_sparse_to_sparse_(Tensor & self, const Tensor & src, bool non_blocking) const override;
};

} // namespace at
