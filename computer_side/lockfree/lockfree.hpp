#ifndef LOCKFREE
#define LOCKFREE

#include <atomic>
#include <cassert>
#include <chrono>
#include <cstddef>
#include <iostream>
#include <thread>
#include <vector>
#include <type_traits> //std::is_trivial
#include <new> //std::hardware_destructive_interference_size

#ifdef __cpp_lib_hardware_interference_size
	using std::hardware_destructive_interference_size;
#else
	// 64 bytes on x86-64 │ L1_CACHE_BYTES │ L1_CACHE_SHIFT │ __cacheline_aligned │ ...
	constexpr std::size_t hardware_destructive_interference_size = 64;
#endif

template <typename t_value>
class ring_buffer
{
public:
	explicit ring_buffer(size_t _capacity) : capacity_(_capacity+1), storage_(_capacity+1)
	{
		assert(_capacity != 0); // Buffer size cannot be 0
	}

	bool push(t_value _value)
	{
		// Here doesn't matter if curr_tail would be
		// loaded earlier of later for other thread,
		// because we the use tail_.load(std::memory_order_acquire)
		// that is creates second barrier 
		const size_t curr_tail = tail_.load(std::memory_order_relaxed);

		// next tail is used multiple times so it
		// have to be saved
		const size_t next_tail = next_index(curr_tail);

		// If buffer is full return to inform user and make him wait
		// Here we create barrier that makes thread read head_ strictly
		// after it stored in other thread's pop

		// We check if last time there was no free space
		if(next_tail == head_cached_)
		{
			// and load actual value only if last time there wasn't any space
			head_cached_ = head_.load(std::memory_order_acquire);

			// to check is situation is changed
			if (next_tail == head_cached_)
			{
				// and if it isn't then we inform user that buffer is full
				return false;
			}
		}

		if constexpr (std::is_trivial<t_value>::value)
		{
			// Copy value to storage (for trivial types compiler cam optimize it itself)
			storage_[curr_tail] = _value;
		}
		else
		{
			// Move value to storage (for non-trivial types)
			storage_[curr_tail] = std::move(_value);
		}

		// Here we create barrier that makes other thread's pop read 
		// tail_ strictly after it stored here
		tail_.store(next_tail, std::memory_order_release);

		// Inform user that value is pushed
		return true;
	}

	bool pop(t_value &_value)
	{
		// Here doesn't matter if curr_head would be
		// loaded earlier of later for other thread,
		// because we the use head_.load(std::memory_order_acquire)
		// that is creates second barrier
		const size_t curr_head = head_.load(std::memory_order_relaxed);

		// We check if last time there was no elements in the buffer
		if(curr_head == tail_cached_)
		{
			// and load actual value only if last time there wasn't any elements
			tail_cached_ = tail_.load(std::memory_order_acquire);

			// to check is situation is changed
			if (curr_head == tail_cached_)
			{
				// and if it isn't then we inform user that buffer is empty
				return false;
			}
		}

		if constexpr (std::is_trivial<t_value>::value)
		{
			// Move data from storage to the _value (for non-trivial types)
			_value = storage_[curr_head];
		}
		else
		{
			// Copy data from storage to the _value (for trivial types compiler cam optimize it itself)
			_value = std::move(storage_[curr_head]);
		}

		// Here we create barrier that makes other thread's push read
		// head_ strictly after it stored here
		head_.store(next_index(curr_head), std::memory_order_release);

		// Inform user that value is pop
		return true;
	}

private:
	inline size_t next_index(const size_t &index) const noexcept
	{
		// Profiling showed that it's most efficient way
		return (index + 1 == capacity_) ? 0 : (index + 1);
	}

// Check achitecture to get cache line size and calculate padding
// we have to add to place head_ and tail_ in different cache lines

	// force head_ and tail_ to different cache lines, to reduce false sharing
	alignas(hardware_destructive_interference_size) std::atomic<size_t> head_{0}; // index of last pushed element of the ring buffer
	alignas(hardware_destructive_interference_size) size_t head_cached_ {0}; 	  // cached index of last pushed element of the ring buffer (to reduce amount of atomic loads)
	alignas(hardware_destructive_interference_size) std::atomic<size_t> tail_{0}; // index of last popped element of the ring buffer (last free element)
	alignas(hardware_destructive_interference_size) size_t tail_cached_{0};		  // cached index of last popped element of the ring buffer (to reduce amount of atomic loads)

	alignas(hardware_destructive_interference_size) const size_t capacity_; // Capacity of the ring buffer
	std::vector<t_value> storage_;									// Storage of the ring buffer
};

#define M_TO_STRING_WRAPPER(x) #x
#define M_TO_STRING(x) M_TO_STRING_WRAPPER(x)
#define M_SOURCE __FILE__ ":" M_TO_STRING(__LINE__)

class stopwatch
{
	using clock_type = std::chrono::steady_clock;

public:
	stopwatch()
	{
		start_ = clock_type::now();
	}

	template <typename t_duration>
	t_duration elapsed_duration() const
	{
		using namespace std::chrono;

		auto delta = clock_type::now() - start_;
		return duration_cast<t_duration>(delta);
	}

private:
	clock_type::time_point start_;
};

class hash_calculator
{
public:
	template <typename t_value>
	void set(const t_value &_value)
	{
		digest_ = std::hash<t_value>()(_value) ^ (digest_ << 1);
	}

	size_t value() const
	{
		return digest_;
	}

private:
	size_t digest_ = 0;
};

// void test()
// {
// 	constexpr size_t k_count = 10'000'000;
// 	constexpr size_t k_size = 1024;

// 	ring_buffer<int> buffer(k_size);

// 	size_t producer_hash = 0;
// 	std::chrono::milliseconds producer_time;

// 	size_t consumer_hash = 0;
// 	std::chrono::milliseconds consumer_time;

// 	std::thread producer([&]()
// 	{
// 		hash_calculator hash;
// 		stopwatch watch;

// 		for (size_t i = 0; i < k_count; ++i) 
// 		{
// 			hash.set(i);

// 			while (!buffer.push(i)) 
// 			{
// 				std::this_thread::yield();
// 			}
// 		}

// 		producer_time = watch.elapsed_duration<std::chrono::milliseconds>();
// 		producer_hash = hash.value(); 
// 	});

// 	std::thread consumer([&]()
// 	{
// 		hash_calculator hash;
// 		stopwatch watch;

// 		for (size_t i = 0; i < k_count; ++i) 
// 		{
// 			int value;

// 			while (!buffer.pop(value)) 
// 			{
// 				std::this_thread::yield();
// 			}

// 			hash.set(value);
// 		}

// 		consumer_time = watch.elapsed_duration<std::chrono::milliseconds>();
// 		consumer_hash = hash.value(); 
// 	});

// 	producer.join();
// 	consumer.join();

// 	if (producer_hash != consumer_hash)
// 	{
// 		throw std::runtime_error(M_SOURCE ": workers hash must be equal");
// 	}

// 	std::cout << "producer_time: " << producer_time.count() << "ms; "
// 			  << "consumer_time: " << consumer_time.count() << "ms"
// 			  << std::endl;
// }

// g++ -std=c++17 -O2 -pthread main.cpp
// g++ -std=c++17 -O2 -pthread -fsanitize=thread main.cpp

#endif