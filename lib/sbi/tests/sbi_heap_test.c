#include <sbi/sbi_unit_test.h>
#include <sbi/sbi_types.h>
#include <sbi/sbi_heap.h>

static u8 zero_mem[1024];

static void test_heap_x_bytes(struct sbiunit_test_case *test, size_t aligned, size_t size)
{
	bool do_aligned_alloc = aligned && !(size & (aligned - 1));
	unsigned long offset, used_space;
	void *ptr1, *ptr2;

	used_space = sbi_heap_used_space();

	if (do_aligned_alloc)
		ptr1 = sbi_aligned_alloc(aligned, size);
	else
		ptr1 = sbi_malloc(size);
	SBIUNIT_EXPECT_NE(test, ptr1, NULL);
	if (do_aligned_alloc) {
		offset = (unsigned long)ptr1 & (aligned - 1);
		SBIUNIT_EXPECT_EQ(test, offset, 0);
	}

	if (ptr1) {
		*(u8 *)ptr1 = 0xAB;
		SBIUNIT_EXPECT_EQ(test, *(u8 *)ptr1, 0xAB);
		*(u8 *)(ptr1 + size / 2) = 0xCD;
		SBIUNIT_EXPECT_EQ(test, *(u8 *)(ptr1 + size / 2), 0xCD);
		*(u8 *)(ptr1 + size - 1) = 0xEF;
		SBIUNIT_EXPECT_EQ(test, *(u8 *)(ptr1 + size - 1), 0xEF);
	}

	ptr2 = sbi_zalloc(size);
	SBIUNIT_EXPECT_NE(test, ptr2, NULL);

	if (ptr2) {
		*(u8 *)ptr2 = 0xAB;
		SBIUNIT_EXPECT_EQ(test, *(u8 *)ptr2, 0xAB);
		*(u8 *)(ptr2 + size / 2) = 0xCD;
		SBIUNIT_EXPECT_EQ(test, *(u8 *)(ptr2 + size / 2), 0xCD);
		*(u8 *)(ptr2 + size - 1) = 0xEF;
		SBIUNIT_EXPECT_EQ(test, *(u8 *)(ptr2 + size - 1), 0xEF);
		*(u8 *)ptr2 = 0x0;
		*(u8 *)(ptr2 + size / 2) = 0x0;
		*(u8 *)(ptr2 + size - 1) = 0x0;
	}

	SBIUNIT_EXPECT_MEMEQ(test, ptr2, zero_mem, size);

	sbi_free(ptr1);
	sbi_free(ptr2);

	SBIUNIT_EXPECT_EQ(test, used_space, sbi_heap_used_space());
}

#define DEFINE_TEST_HEAP_X_BYTES(__x)					\
static void test_heap_bytes_##__x(struct sbiunit_test_case *test)	\
{									\
	test_heap_x_bytes(test, 0, __x);				\
}

DEFINE_TEST_HEAP_X_BYTES(1)
DEFINE_TEST_HEAP_X_BYTES(2)
DEFINE_TEST_HEAP_X_BYTES(3)
DEFINE_TEST_HEAP_X_BYTES(4)
DEFINE_TEST_HEAP_X_BYTES(5)
DEFINE_TEST_HEAP_X_BYTES(8)
DEFINE_TEST_HEAP_X_BYTES(12)
DEFINE_TEST_HEAP_X_BYTES(13)
DEFINE_TEST_HEAP_X_BYTES(16)
DEFINE_TEST_HEAP_X_BYTES(24)
DEFINE_TEST_HEAP_X_BYTES(27)
DEFINE_TEST_HEAP_X_BYTES(32)
DEFINE_TEST_HEAP_X_BYTES(48)
DEFINE_TEST_HEAP_X_BYTES(59)
DEFINE_TEST_HEAP_X_BYTES(64)
DEFINE_TEST_HEAP_X_BYTES(96)
DEFINE_TEST_HEAP_X_BYTES(111)
DEFINE_TEST_HEAP_X_BYTES(128)
DEFINE_TEST_HEAP_X_BYTES(192)
DEFINE_TEST_HEAP_X_BYTES(225)
DEFINE_TEST_HEAP_X_BYTES(256)
DEFINE_TEST_HEAP_X_BYTES(384)
DEFINE_TEST_HEAP_X_BYTES(447)
DEFINE_TEST_HEAP_X_BYTES(512)
DEFINE_TEST_HEAP_X_BYTES(768)
DEFINE_TEST_HEAP_X_BYTES(911)
DEFINE_TEST_HEAP_X_BYTES(1024)

#define DEFINE_TEST_HEAP_ALIGNED64_X_BYTES(__x)					\
static void test_heap_aligned64_bytes_##__x(struct sbiunit_test_case *test)	\
{										\
	test_heap_x_bytes(test, 64, __x);					\
}

DEFINE_TEST_HEAP_ALIGNED64_X_BYTES(64)
DEFINE_TEST_HEAP_ALIGNED64_X_BYTES(128)
DEFINE_TEST_HEAP_ALIGNED64_X_BYTES(192)
DEFINE_TEST_HEAP_ALIGNED64_X_BYTES(256)
DEFINE_TEST_HEAP_ALIGNED64_X_BYTES(320)
DEFINE_TEST_HEAP_ALIGNED64_X_BYTES(384)
DEFINE_TEST_HEAP_ALIGNED64_X_BYTES(448)
DEFINE_TEST_HEAP_ALIGNED64_X_BYTES(512)
DEFINE_TEST_HEAP_ALIGNED64_X_BYTES(576)
DEFINE_TEST_HEAP_ALIGNED64_X_BYTES(640)
DEFINE_TEST_HEAP_ALIGNED64_X_BYTES(704)
DEFINE_TEST_HEAP_ALIGNED64_X_BYTES(768)
DEFINE_TEST_HEAP_ALIGNED64_X_BYTES(832)
DEFINE_TEST_HEAP_ALIGNED64_X_BYTES(896)
DEFINE_TEST_HEAP_ALIGNED64_X_BYTES(960)
DEFINE_TEST_HEAP_ALIGNED64_X_BYTES(1024)

#define DEFINE_TEST_HEAP_ALIGNED128_X_BYTES(__x)				\
static void test_heap_aligned128_bytes_##__x(struct sbiunit_test_case *test)	\
{										\
	test_heap_x_bytes(test, 128, __x);					\
}

DEFINE_TEST_HEAP_ALIGNED128_X_BYTES(128)
DEFINE_TEST_HEAP_ALIGNED128_X_BYTES(256)
DEFINE_TEST_HEAP_ALIGNED128_X_BYTES(384)
DEFINE_TEST_HEAP_ALIGNED128_X_BYTES(512)
DEFINE_TEST_HEAP_ALIGNED128_X_BYTES(640)
DEFINE_TEST_HEAP_ALIGNED128_X_BYTES(768)
DEFINE_TEST_HEAP_ALIGNED128_X_BYTES(896)
DEFINE_TEST_HEAP_ALIGNED128_X_BYTES(1024)

#define DEFINE_TEST_HEAP_ALIGNED256_X_BYTES(__x)				\
static void test_heap_aligned256_bytes_##__x(struct sbiunit_test_case *test)	\
{										\
	test_heap_x_bytes(test, 256, __x);					\
}

DEFINE_TEST_HEAP_ALIGNED256_X_BYTES(256)
DEFINE_TEST_HEAP_ALIGNED256_X_BYTES(512)
DEFINE_TEST_HEAP_ALIGNED256_X_BYTES(768)
DEFINE_TEST_HEAP_ALIGNED256_X_BYTES(1024)

#define DEFINE_TEST_HEAP_ALIGNED512_X_BYTES(__x)				\
static void test_heap_aligned512_bytes_##__x(struct sbiunit_test_case *test)	\
{										\
	test_heap_x_bytes(test, 512, __x);					\
}

DEFINE_TEST_HEAP_ALIGNED512_X_BYTES(512)
DEFINE_TEST_HEAP_ALIGNED512_X_BYTES(1024)

static struct sbiunit_test_case heap_tests[] = {
	SBIUNIT_TEST_CASE(test_heap_bytes_1),
	SBIUNIT_TEST_CASE(test_heap_bytes_2),
	SBIUNIT_TEST_CASE(test_heap_bytes_3),
	SBIUNIT_TEST_CASE(test_heap_bytes_4),
	SBIUNIT_TEST_CASE(test_heap_bytes_5),
	SBIUNIT_TEST_CASE(test_heap_bytes_8),
	SBIUNIT_TEST_CASE(test_heap_bytes_12),
	SBIUNIT_TEST_CASE(test_heap_bytes_13),
	SBIUNIT_TEST_CASE(test_heap_bytes_16),
	SBIUNIT_TEST_CASE(test_heap_bytes_24),
	SBIUNIT_TEST_CASE(test_heap_bytes_27),
	SBIUNIT_TEST_CASE(test_heap_bytes_32),
	SBIUNIT_TEST_CASE(test_heap_bytes_48),
	SBIUNIT_TEST_CASE(test_heap_bytes_59),
	SBIUNIT_TEST_CASE(test_heap_bytes_64),
	SBIUNIT_TEST_CASE(test_heap_bytes_96),
	SBIUNIT_TEST_CASE(test_heap_bytes_111),
	SBIUNIT_TEST_CASE(test_heap_bytes_128),
	SBIUNIT_TEST_CASE(test_heap_bytes_192),
	SBIUNIT_TEST_CASE(test_heap_bytes_225),
	SBIUNIT_TEST_CASE(test_heap_bytes_256),
	SBIUNIT_TEST_CASE(test_heap_bytes_384),
	SBIUNIT_TEST_CASE(test_heap_bytes_447),
	SBIUNIT_TEST_CASE(test_heap_bytes_512),
	SBIUNIT_TEST_CASE(test_heap_bytes_768),
	SBIUNIT_TEST_CASE(test_heap_bytes_911),
	SBIUNIT_TEST_CASE(test_heap_bytes_1024),
	SBIUNIT_TEST_CASE(test_heap_aligned64_bytes_64),
	SBIUNIT_TEST_CASE(test_heap_aligned64_bytes_128),
	SBIUNIT_TEST_CASE(test_heap_aligned64_bytes_192),
	SBIUNIT_TEST_CASE(test_heap_aligned64_bytes_256),
	SBIUNIT_TEST_CASE(test_heap_aligned64_bytes_320),
	SBIUNIT_TEST_CASE(test_heap_aligned64_bytes_384),
	SBIUNIT_TEST_CASE(test_heap_aligned64_bytes_448),
	SBIUNIT_TEST_CASE(test_heap_aligned64_bytes_512),
	SBIUNIT_TEST_CASE(test_heap_aligned64_bytes_576),
	SBIUNIT_TEST_CASE(test_heap_aligned64_bytes_640),
	SBIUNIT_TEST_CASE(test_heap_aligned64_bytes_704),
	SBIUNIT_TEST_CASE(test_heap_aligned64_bytes_768),
	SBIUNIT_TEST_CASE(test_heap_aligned64_bytes_832),
	SBIUNIT_TEST_CASE(test_heap_aligned64_bytes_896),
	SBIUNIT_TEST_CASE(test_heap_aligned64_bytes_960),
	SBIUNIT_TEST_CASE(test_heap_aligned64_bytes_1024),
	SBIUNIT_TEST_CASE(test_heap_aligned128_bytes_128),
	SBIUNIT_TEST_CASE(test_heap_aligned128_bytes_256),
	SBIUNIT_TEST_CASE(test_heap_aligned128_bytes_384),
	SBIUNIT_TEST_CASE(test_heap_aligned128_bytes_512),
	SBIUNIT_TEST_CASE(test_heap_aligned128_bytes_640),
	SBIUNIT_TEST_CASE(test_heap_aligned128_bytes_768),
	SBIUNIT_TEST_CASE(test_heap_aligned128_bytes_896),
	SBIUNIT_TEST_CASE(test_heap_aligned128_bytes_1024),
	SBIUNIT_TEST_CASE(test_heap_aligned256_bytes_256),
	SBIUNIT_TEST_CASE(test_heap_aligned256_bytes_512),
	SBIUNIT_TEST_CASE(test_heap_aligned256_bytes_768),
	SBIUNIT_TEST_CASE(test_heap_aligned256_bytes_1024),
	SBIUNIT_TEST_CASE(test_heap_aligned512_bytes_512),
	SBIUNIT_TEST_CASE(test_heap_aligned512_bytes_1024),
	SBIUNIT_END_CASE,
};

SBIUNIT_TEST_SUITE(heap_test_suite, heap_tests);
