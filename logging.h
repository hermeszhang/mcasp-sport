/* -*- Mode: C; tab-width: 8; indent-tabs-mode: t; c-basic-offset: 8 -*- */
#ifndef GTSPORT_LOGGING_H
#define GTSPORT_LOGGING_H

#define ENABLE_DEBUG_LOGGING

#ifdef ENABLE_DEBUG_LOGGING
static inline
const char *_basename(const char *path) {
	const char *result = path;
	while (*path) {
		if (*path++ == '/')
			result = path;
	}
	return result;
}

#define DEBUG(fmt, args...)						\
	printk(/*KERN_DEBUG*/ "%s:%d:%s(): " fmt "\n",			\
	       _basename(__FILE__), __LINE__, __func__, ## args)
#else
#define DEBUG(fmt, ...) do {} while (0)
#endif

#define ERROR(fmt, args...)						\
	printk(KERN_ERR "%s:%d:%s(): " fmt "\n",			\
	       _basename(__FILE__), __LINE__, __func__, ## args)
#define WARNING(fmt, args...)						\
	printk(KERN_WARNING "%s:%d:%s(): " fmt "\n",			\
	       _basename(__FILE__), __LINE__, __func__, ## args)
#endif
