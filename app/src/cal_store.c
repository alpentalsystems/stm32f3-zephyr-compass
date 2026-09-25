#include <errno.h>
#include <stdbool.h>
#include <string.h>
#include <zephyr/settings/settings.h>

#include "cal_store.h"

#define KEY_LEAF "mag_offset"

static struct vec3 loaded_offset;
static bool found;

static int compass_settings_set(const char *key, size_t len, settings_read_cb read_cb,
				void *cb_arg)
{
	ssize_t n;

	if (strcmp(key, KEY_LEAF) != 0) {
		return -ENOENT;
	}
	if (len != sizeof(loaded_offset)) {
		return -EINVAL;
	}
	n = read_cb(cb_arg, &loaded_offset, sizeof(loaded_offset));
	if (n < 0) {
		return (int)n;
	}
	if (n != (ssize_t)sizeof(loaded_offset)) {
		return -EIO;
	}
	found = true;
	return 0;
}

SETTINGS_STATIC_HANDLER_DEFINE(compass, "compass", NULL, compass_settings_set, NULL, NULL);

int cal_store_load(struct vec3 *offset)
{
	int err = settings_subsys_init();

	if (err != 0) {
		return err;
	}
	found = false;
	err = settings_load_subtree("compass");
	if (err != 0) {
		return err;
	}
	if (!found) {
		return -ENOENT;
	}
	*offset = loaded_offset;
	return 0;
}

int cal_store_save(const struct vec3 *offset)
{
	return settings_save_one("compass/" KEY_LEAF, offset, sizeof(*offset));
}
