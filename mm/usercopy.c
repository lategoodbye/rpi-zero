/*
 * This implements the various checks for CONFIG_HARDENED_USERCOPY*,
 * which are designed to protect kernel memory from needless exposure
 * and overwrite under many unintended conditions. This code is based
 * on PAX_USERCOPY, which is:
 *
 * Copyright (C) 2001-2016 PaX Team, Bradley Spengler, Open Source
 * Security Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/mm.h>
#include <linux/slab.h>
#include <asm/sections.h>

enum {
	BAD_STACK = -1,
	NOT_STACK = 0,
	GOOD_FRAME,
	GOOD_STACK,
};

/*
 * Checks if a given pointer and length is contained by the current
 * stack frame (if possible).
 *
 *	0: not at all on the stack
 *	1: fully within a valid stack frame
 *	2: fully on the stack (when can't do frame-checking)
 *	-1: error condition (invalid stack position or bad stack frame)
 */
static noinline int check_stack_object(const void *obj, unsigned long len)
{
	const void * const stack = task_stack_page(current);
	const void * const stackend = stack + THREAD_SIZE;
	int ret;

	/* Object is not on the stack at all. */
	if (obj + len <= stack || stackend <= obj)
		return NOT_STACK;

	/*
	 * Reject: object partially overlaps the stack (passing the
	 * the check above means at least one end is within the stack,
	 * so if this check fails, the other end is outside the stack).
	 */
	if (obj < stack || stackend < obj + len)
		return BAD_STACK;

	/* Check if object is safely within a valid frame. */
	ret = arch_within_stack_frames(stack, stackend, obj, len);
	if (ret)
		return ret;

	return GOOD_STACK;
}

static void report_usercopy(const void *ptr, unsigned long len,
			    bool to_user, const char *type)
{
	pr_emerg("kernel memory %s attempt detected %s %p (%s) (%lu bytes)\n",
		to_user ? "exposure" : "overwrite",
		to_user ? "from" : "to", ptr, type ? : "unknown", len);
	/*
	 * For greater effect, it would be nice to do do_group_exit(),
	 * but BUG() actually hooks all the lock-breaking and per-arch
	 * Oops code, so that is used here instead.
	 */
	BUG();
}

/* Returns true if any portion of [ptr,ptr+n) over laps with [low,high). */
static bool overlaps(const void *ptr, unsigned long n, unsigned long low,
		     unsigned long high)
{
	unsigned long check_low = (uintptr_t)ptr;
	unsigned long check_high = check_low + n;

	/* Does not overlap if entirely above or entirely below. */
	if (check_low >= high || check_high < low)
		return false;

	return true;
}

/* Is this address range in the kernel text area? */
static inline const char *check_kernel_text_object(const void *ptr,
						   unsigned long n)
{
	unsigned long textlow = (unsigned long)_stext;
	unsigned long texthigh = (unsigned long)_etext;

	if (overlaps(ptr, n, textlow, texthigh))
		return "<kernel text>";

#ifdef HAVE_ARCH_LINEAR_KERNEL_MAPPING
	/* Check against linear mapping as well. */
	if (overlaps(ptr, n, (unsigned long)__va(__pa(textlow)),
		     (unsigned long)__va(__pa(texthigh))))
		return "<linear kernel text>";
#endif

	return NULL;
}

static inline const char *check_bogus_address(const void *ptr, unsigned long n)
{
	/* Reject if object wraps past end of memory. */
	if (ptr + n < ptr)
		return "<wrapped address>";

	/* Reject if NULL or ZERO-allocation. */
	if (ZERO_OR_NULL_PTR(ptr))
		return "<null>";

	return NULL;
}

static inline const char *check_heap_object(const void *ptr, unsigned long n,
					    bool to_user)
{
	struct page *page, *endpage;
	const void *end = ptr + n - 1;

	if (!virt_addr_valid(ptr))
		return NULL;

	page = virt_to_head_page(ptr);

	/* Check slab allocator for flags and size. */
	if (PageSlab(page))
		return __check_heap_object(ptr, n, page);

	/*
	 * Sometimes the kernel data regions are not marked Reserved (see
	 * check below). And sometimes [_sdata,_edata) does not cover
	 * rodata and/or bss, so check each range explicitly.
	 */

	/* Allow reads of kernel rodata region (if not marked as Reserved). */
	if (ptr >= (const void *)__start_rodata &&
	    end <= (const void *)__end_rodata) {
		if (!to_user)
			return "<rodata>";
		return NULL;
	}

	/* Allow kernel data region (if not marked as Reserved). */
	if (ptr >= (const void *)_sdata && end <= (const void *)_edata)
		return NULL;

	/* Allow kernel bss region (if not marked as Reserved). */
	if (ptr >= (const void *)__bss_start &&
	    end <= (const void *)__bss_stop)
		return NULL;

	/* Is the object wholly within one base page? */
	if (likely(((unsigned long)ptr & (unsigned long)PAGE_MASK) ==
		   ((unsigned long)end & (unsigned long)PAGE_MASK)))
		return NULL;

	/* Allow if start and end are inside the same compound page. */
	endpage = virt_to_head_page(end);
	if (likely(endpage == page))
		return NULL;

	/*
	 * Reject if range is not Reserved (i.e. special or device memory),
	 * since then the object spans several independently allocated pages.
	 */
	for (; ptr <= end ; ptr += PAGE_SIZE, page = virt_to_head_page(ptr)) {
		if (!PageReserved(page))
			return "<spans multiple pages>";
	}

	return NULL;
}

/*
 * Validates that the given object is one of:
 * - known safe heap object
 * - known safe stack object
 * - not in kernel text
 */
void __check_object_size(const void *ptr, unsigned long n, bool to_user)
{
	const char *err;

	/* Skip all tests if size is zero. */
	if (!n)
		return;

	/* Check for invalid addresses. */
	err = check_bogus_address(ptr, n);
	if (err)
		goto report;

	/* Check for bad heap object. */
	err = check_heap_object(ptr, n, to_user);
	if (err)
		goto report;

	/* Check for bad stack object. */
	switch (check_stack_object(ptr, n)) {
	case NOT_STACK:
		/* Object is not touching the current process stack. */
		break;
	case GOOD_FRAME:
	case GOOD_STACK:
		/*
		 * Object is either in the correct frame (when it
		 * is possible to check) or just generally on the
		 * process stack (when frame checking not available).
		 */
		return;
	default:
		err = "<process stack>";
		goto report;
	}

	/* Check for object in kernel to avoid text exposure. */
	err = check_kernel_text_object(ptr, n);
	if (!err)
		return;

report:
	report_usercopy(ptr, n, to_user, err);
}
EXPORT_SYMBOL(__check_object_size);
