#include <CUDA/cudaCommon.cuh>

namespace Time
{
	chrono::steady_clock::time_point Now()
	{
		return chrono::high_resolution_clock::now();
	}

	uint64_t Microseconds(chrono::steady_clock::time_point& from, chrono::steady_clock::time_point& now)
	{
		return std::chrono::duration_cast<std::chrono::microseconds>(now - from).count();
	}

	chrono::steady_clock::time_point End(chrono::steady_clock::time_point& from, const string& message, int number)
	{
		auto now = chrono::high_resolution_clock::now();
		if (-1 == number)
		{
			printf("[%s] %.4f ms from start\n", message.c_str(), (float)(Microseconds(from, now)) / 1000.0f);
		}
		else
		{
			printf("[%6d - %s] %.4f ms from start\n", number, message.c_str(), (float)(Microseconds(from, now)) / 1000.0f);
		}
		return now;
	}

	string DateTime()
	{
		auto t = std::time(nullptr);
		auto tm = *std::localtime(&t);

		std::ostringstream oss;
		oss << std::put_time(&tm, "%Y%m%d_%H%M%S"); // Format: YYYYMMDD_HHMMSS
		return oss.str();
	}
}

string Miliseconds(const chrono::steady_clock::time_point beginTime, const char* tag)
{
	auto now = chrono::high_resolution_clock::now();
	auto timeSpan = chrono::duration_cast<chrono::nanoseconds>(now - beginTime).count();
	stringstream ss;
	ss << "[[[ ";
	if (nullptr != tag)
	{
		ss << tag << " - ";
	}
	ss << (float)timeSpan / 1000000.0 << " ms ]]]";
	return ss.str();
}

__host__ __device__ float3 operator+(const float3& a, const float3& b)
{
    return make_float3(a.x + b.x, a.y + b.y, a.z + b.z);
}

__host__ __device__ float3& operator+=(float3& a, const float3& b)
{
	a.x += b.x;  a.y += b.y;  a.z += b.z;  return a;
}

__host__ __device__ float3 operator-(const float3& a, const float3& b)
{
	return make_float3(a.x - b.x, a.y - b.y, a.z - b.z);
}

__host__ __device__ float3& operator-=(float3& a, const float3& b)
{
	a.x -= b.x;  a.y -= b.y;  a.z -= b.z;  return a;
}

__host__ __device__ float3 operator*(const float3& a, float b)
{
    return make_float3(a.x * b, a.y * b, a.z * b);
}

__host__ __device__ float3& operator*=(float3& a, float b)
{
	a.x *= b;  a.y *= b;  a.z *= b;  return a;
}

__host__ __device__ float3 operator*(float b, const float3& a)
{
	return make_float3(a.x * b, a.y * b, a.z * b);
}

__host__ __device__ float3& operator*=(float b, float3& a)
{
	a.x *= b;  a.y *= b;  a.z *= b;  return a;
}

__host__ __device__ float3 operator/(const float3& a, float b)
{
	return make_float3(a.x / b, a.y / b, a.z / b);
}

__host__ __device__ float3& operator/=(float3& a, float b)
{
	a.x /= b;  a.y /= b;  a.z /= b;  return a;
}

__host__ __device__ float3 operator/(float b, const float3& a)
{
	return make_float3(a.x / b, a.y / b, a.z / b);
}

__host__ __device__ float3& operator/=(float b, float3& a)
{
	a.x /= b;  a.y /= b;  a.z /= b;  return a;
}

__host__ __device__ float dot(const float3& a, const float3& b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

__host__ __device__ float3 cross(const float3& a, const float3& b)
{
	return make_float3(
		a.y * b.z - a.z * b.y,
		a.z * b.x - a.x * b.z,
		a.x * b.y - a.y * b.x
	);
}

__host__ __device__ float length(const float3& v)
{
	return sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
}

__host__ __device__ float lengthSquared(const float3& v)
{
	return v.x * v.x + v.y * v.y + v.z * v.z;
}

__host__ __device__ float3 normalize(const float3& v)
{
	float len = length(v);
	if (len > 0.f)
		return make_float3(v.x / len, v.y / len, v.z / len);
	else
		return make_float3(0.f, 0.f, 0.f);  // 또는 처리 방식 선택
}

__device__ __host__ float hashToFloat(uint32_t seed)
{
    seed ^= seed >> 13;
    seed *= 0x5bd1e995;
    seed ^= seed >> 15;
    return (seed & 0xFFFFFF) / static_cast<float>(0xFFFFFF);
};

__device__ __host__ size_t voxel_hash(int3 coord, size_t tableSize)
{
    return ((size_t)(coord.x * 73856093) ^ (coord.y * 19349663) ^ (coord.z * 83492791)) % tableSize;
}

__device__ float atomicMinFloat(float* address, float value)
{
	int* addr_as_int = (int*)address;
	int old = *addr_as_int, assumed;

	do {
		assumed = old;
		float old_value = __int_as_float(assumed);
		if (value >= old_value) break;  // 이미 더 큰 값이면 갱신 안 함

		old = atomicCAS(addr_as_int, assumed, __float_as_int(value));
	} while (assumed != old);

	return __int_as_float(old);
}
