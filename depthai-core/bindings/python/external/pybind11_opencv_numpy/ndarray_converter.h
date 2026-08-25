# ifndef __NDARRAY_CONVERTER_H__
# define __NDARRAY_CONVERTER_H__

#include <Python.h>
#include <opencv2/core/core.hpp>
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <numpy/ndarrayobject.h>

class NumpyAllocator : public cv::MatAllocator
{
public:
    NumpyAllocator();
    ~NumpyAllocator();

    cv::UMatData* allocate(PyObject* o, int dims, const int* sizes, int type, size_t* step) const;

#if CV_MAJOR_VERSION < 4
    cv::UMatData* allocate(int dims0, const int* sizes, int type, void* data, size_t* step, int flags, cv::UMatUsageFlags usageFlags) const;
#else
    cv::UMatData* allocate(int dims0, const int* sizes, int type, void* data, size_t* step, cv::AccessFlag flags, cv::UMatUsageFlags usageFlags) const;
#endif

#if CV_MAJOR_VERSION < 4
    bool allocate(cv::UMatData* u, int accessFlags, cv::UMatUsageFlags usageFlags) const;
#else
    bool allocate(cv::UMatData* u, cv::AccessFlag accessFlags, cv::UMatUsageFlags usageFlags) const;
#endif

    void deallocate(cv::UMatData* u) const;

private:
    const cv::MatAllocator* stdAllocator;
};
extern NumpyAllocator g_numpyAllocator;

class NDArrayConverter {
public:
    // must call this first, or the other routines don't work!
    static bool init_numpy();

    static bool toMat(PyObject* o, cv::Mat &m);
    static PyObject* toNDArray(const cv::Mat& mat);
};

//
// Define the type converter
//

#include <pybind11/pybind11.h>

namespace pybind11 { namespace detail {

template <> struct type_caster<cv::Mat> {
public:

    PYBIND11_TYPE_CASTER(cv::Mat, _("numpy.ndarray"));

    bool load(handle src, bool /* convert */) {
        const bool converted = NDArrayConverter::toMat(src.ptr(), value);
        if (!converted) {
            // A failed type-caster load asks pybind11 to try the next overload.
            // Do not leak the converter's diagnostic exception into a later
            // overload that accepts the argument successfully.
            PyErr_Clear();
        }
        return converted;
    }

    static handle cast(const cv::Mat &m, return_value_policy, handle defval) {
        return handle(NDArrayConverter::toNDArray(m));
    }
};


}} // namespace pybind11::detail

# endif
