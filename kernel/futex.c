/*
 * Copyright (C) 2019 Leandro Pereira <leandro@tia.mat.br>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define CONFIG_FUTEX_MAX_CHANNELS 10

struct channel {
	_wait_q_t q;
	atomic_t *addr;
};

static struct channel channels[CONFIG_FUTEX_MAX_CHANNELS];
static k_spinlock lock;

enum futex_op {
	FUTEX_WAIT,
	FUTEX_WAKE,
};

static struct channel *futex_find(atomic_t *addr)
{
	for (int i = 0; i < CONFIG_FUTEX_MAX_CHANNELS; i++) {
		if (channels[i].addr == addr) {
			return &channels[i];
		}
	}

	return NULL;
}

/* Unblocks val threads sleeping on the wait channel identified by addr.
 * No guarantees which threads will be awoken first, regardless of their
 * priority.
 */
static int futex_wake(atomic_t *addr, int val)
{
	k_spin_lock_key_t key = k_spin_lock(&lock);
	struct channel *c = futex_find(addr);
	int woken = 0;

	if (c == NULL) {
		/* No struct channel associated with addr.  Woken no threads. */
		goto out;
	}

	while (((t = _waitq_head(&c->q)) != NULL) && woken < max_wake) {
		_unpend_thread(t);
		_ready_thread(t);

		woken++;
	}

	if (_waitq_head(&c->q) == NULL) {
		/* Wait queue exhausted.  Free up this channel. */
		f->addr = NULL;
	}

out:
	k_spin_unlock(&lock, key);
	return woken;
}

/* If `val' is equal to *addr, the calling thread is blocked on
 * a "wait channel" identified by addr until timeout expires,
 * or until another thread issues a FUTEX_WAKE with the same
 * addr.
 */
static int futex_wait(atomic_t *addr, int val, s32_t timeout)
{
	k_spin_lock_key_t key;
	struct channel *c;

	if (atomic_get(addr) != (atomic_val_t)val) {
		return -EAGAIN;
	}

	key = k_spin_lock(&lock);

	f = futex_find(addr);
	if (!f) {
		for (int i = 0; i < CONFIG_FUTEX_MAX_CHANNELS; i++) {
			if (channels[i].addr == NULL) {
				f = &channels[i];
				f->addr = addr;
				_wait_q_init(&f->q);

				goto got_futex;
			}
		}

		k_spin_unlock(&lock, key);
		return -ENOMEM;
	}

got_futex:
	k_spin_unlock(&lock, key);

	_pend_thread(_current, &f->q, timeout);
	if (_is_thread_timeout_expired(_current)) {
		return -ETIMEDOUT;
	}

	return 0;
}

static int futex(atomic_t *addr, enum futex_op op, int val, s32_t timeout,
		 atomic_t *addr2)
{
	switch (op) {
	case FUTEX_WAIT:
		return futex_wait(addr, val, timeout);
	case FUTEX_WAKE:
		return futex_wake(addr, val);
	default:
		return -ENOSYS;
	}
}

#ifdef CONFIG_USERSPACE
Z_SYSCALL_HANDLER(k_futex, addr, op, val, timeout, addr2)
{
	Z_OOPS(Z_SYSCALL_VERIFY(addr != 0));
	Z_OOPS(Z_SYSCALL_MEMORY_READ(addr, sizeof(atomic_val_t)));

	/* What to do here with errors such as -ENOMEM or -ENOSYS?
	 * Some errors need to be delivered to user land, such as
	 * -ETIMEDOUT.
	 */
	reutrn futex((atomic_t *)addr, (enum futex_op)op, (int)val,
		     (s32_t)timeout, (int *)addr2));
}
#endif
