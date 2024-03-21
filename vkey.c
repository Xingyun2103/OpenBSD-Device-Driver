#include <sys/param.h>
#include <sys/systm.h>
#include <sys/errno.h>
#include <sys/ioctl.h>
#include <sys/device.h>
#include <sys/vnode.h>
#include <sys/atomic.h>
#include <sys/malloc.h>

#include <machine/bus.h>

#include <dev/pci/pcireg.h>
#include <dev/pci/pcivar.h>
#include <dev/pci/pcidevs.h>
#include <dev/vkeyvar.h>

#define VKEY_BAR	0x10

#define VKEY_VMAJ	0x00
#define VKEY_VMIN	0x04
#define VKEY_FLAGS	0x08
#define VKEY_CBASE	0x10
#define VKEY_CSHIFT	0x18
#define VKEY_RBASE	0x20
#define VKEY_RSHIFT	0x28
#define VKEY_CPBASE	0x30
#define VKEY_CPSHIFT	0x38
#define VKEY_DBELL	0x40
#define VKEY_CPDBELL	0x44
#define DEVICE_OWNER	0xAA
#define HOST_OWNER	0x55

#define VKEY_READY	0
#define VKEY_BUSY	1
#define VKEY_DONE	2

static int	vkey_match(struct device *, void *, void *);
static void	vkey_attach(struct device *, struct device *, void *);

struct cr_desc {
	uint8_t		owner;
	uint8_t		type;
	uint8_t		reserved[6];

	int		length1;
	int		length2;
	int		length3;
	int		length4;

	unsigned long	cookie;
	void*		pointer1;
	void*		pointer2;
	void*		pointer3;
	void*		pointer4;
};

struct vkey_mem {
	bus_dmamap_t		map;
	bus_dma_segment_t	*segs;
	int			nsegs;
	caddr_t			kva;
};

struct cp_desc {
	uint8_t		owner;
	uint8_t		type;
	uint8_t		reserved[6];

	int		msglen;
	int		reserved2;

	unsigned long	cmdcookie;
	unsigned long	rpcookie;
};


struct entry {
	bool			finished;
	struct vkey_cmd_arg	*arg;
	int			index;
	int			rindex;
	int			error;
	bus_dmamap_t		rmap;

	unsigned long		cookie;
	int			length;
	uint8_t			rtype;
	int			rlen;
	caddr_t			rpointer;
	void*			pointer;

	SLIST_ENTRY(entry)	entries;
};

struct vkey_softc {
	struct device	 sc_dev;
	pci_chipset_tag_t	sc_pc;
	pci_intr_handle_t	sc_ih0;
	pci_intr_handle_t	sc_ih1;
	void			*sc_ihc0;
	void			*sc_ihc1;
	pcitag_t		sc_tag;

	bus_dma_tag_t		sc_dmat;
	bus_space_tag_t		sc_memt;
	bus_space_handle_t	sc_memh;
	bus_size_t		sc_mems;

	bus_dmamap_t		sc_dmap_cbase;
	bus_dmamap_t		sc_dmap_rbase;
	bus_dmamap_t		sc_dmap_cpbase;

	struct cr_desc		*sc_cbase;
	struct cr_desc		*sc_rbase;
	struct cp_desc		*sc_cpbase;

	struct mutex		sc_mtx;
	unsigned int		sc_state;
	int			index;
	int			ints;
	SLIST_HEAD(listhead, entry)	cmds;
};

const struct cfattach vkey_ca = {
	.ca_devsize 	= sizeof(struct vkey_softc),
	.ca_match 	= vkey_match,
	.ca_attach 	= vkey_attach
};

struct cfdriver vkey_cd = {
	.cd_devs 	= NULL,
	.cd_name	= "vkey",
	.cd_class	= DV_DULL
};

static int
vkey_match(struct device *parent, void *match, void *aux)
{
	struct pci_attach_args *pa = aux;
	if (PCI_VENDOR(pa->pa_id) == 0x3301 &&
	    PCI_PRODUCT(pa->pa_id) == 0x0200)
		return (1);
	return (0);
}

void *
vkey_dma_allocmem(struct vkey_softc *sc, u_int size, u_int align,
    bus_dmamap_t *map)
{
	bus_dma_tag_t t = sc->sc_dmat;
	bus_dma_segment_t segs[1];
	caddr_t va;
	int n;

	if (bus_dmamem_alloc(t, size, align, 0, segs, 1, &n, BUS_DMA_WAITOK))
		return NULL;
	if (bus_dmamem_map(t, segs, 1, size, &va, BUS_DMA_WAITOK))
		return NULL;
	if (bus_dmamap_create(t, size, 1, size, 0, BUS_DMA_WAITOK, map))
		return NULL;
	if (bus_dmamap_load_raw(t, *map, segs, n, size, BUS_DMA_WAITOK))
		return NULL;
	bzero(va, size);
	return va;
}

void *
vkey_dma_allocmem2(struct vkey_softc *sc, u_int size, u_int align,
    bus_dmamap_t *map, bus_dma_segment_t *segs, int *n)
{
	bus_dma_tag_t t = sc->sc_dmat;
	caddr_t va;

	if (bus_dmamap_create(t, size, 1, size, 0,
	    BUS_DMA_WAITOK | BUS_DMA_ALLOCNOW, map))
		return NULL;
	if (bus_dmamem_alloc(t, size, align, 0, segs, 1, n, BUS_DMA_WAITOK))
		return NULL;
	if (bus_dmamap_load_raw(t, *map, segs, *n, size, BUS_DMA_WAITOK))
		return NULL;
	if (bus_dmamem_map(t, segs, 1, size, &va, BUS_DMA_WAITOK))
		return NULL;

	bzero(va, size);
	return va;
}


static int	vkey_intr(void *);
static int	vkey_fail(void *);

static void
vkey_attach(struct device *parent, struct device *self, void *aux)
{
	struct vkey_softc *sc = (struct vkey_softc *)self;
	struct pci_attach_args *pa = aux;
	pcireg_t memtype;

	sc->sc_pc = pa->pa_pc;
	sc->sc_tag = pa->pa_tag;
	sc->sc_dmat = pa->pa_dmat;
	sc->index = 0;

	SLIST_INIT(&sc->cmds);

	mtx_init(&sc->sc_mtx, IPL_BIO);

	memtype = pci_mapreg_type(sc->sc_pc, sc->sc_tag, VKEY_BAR);
	if (pci_mapreg_map(pa, VKEY_BAR, memtype, 0, &sc->sc_memt,
	    &sc->sc_memh, NULL, &sc->sc_mems, 0)) {
		printf(": unable to map register memory\n");
		return;
	}

	if (sc->sc_mems < 0x48) {
		printf(": register window is too small\n");
		goto unmap;
	}

	sc->sc_cbase = (struct cr_desc *)vkey_dma_allocmem(sc, PAGE_SIZE,
	    PAGE_SIZE, &sc->sc_dmap_cbase);
	if (sc->sc_cbase == NULL) {
		printf("failed to map command ring\n");
		return;
	}

	sc->sc_rbase = (struct cr_desc *)vkey_dma_allocmem(sc, PAGE_SIZE,
	    PAGE_SIZE, &sc->sc_dmap_rbase);
	if (sc->sc_rbase == NULL) {
		printf("failed to map reply ring\n");
		return;
	}

	sc->sc_cpbase = (struct cp_desc *)vkey_dma_allocmem(sc, PAGE_SIZE/2,
	    PAGE_SIZE/2, &sc->sc_dmap_cpbase);
	if (sc->sc_cpbase == NULL) {
		printf("failed to map completion ring\n");
		return;
	}

	for (int i = 0; i < 64; i++) {
		sc->sc_cbase[i].owner = HOST_OWNER;
		sc->sc_rbase[i].owner = HOST_OWNER;
		sc->sc_cpbase[i].owner = DEVICE_OWNER;
	}

	bus_space_write_8(sc->sc_memt, sc->sc_memh, 0x10,
	    sc->sc_dmap_cbase->dm_segs[0].ds_addr);
	bus_space_write_8(sc->sc_memt, sc->sc_memh, 0x20,
	    sc->sc_dmap_rbase->dm_segs[0].ds_addr);
	bus_space_write_8(sc->sc_memt, sc->sc_memh, 0x30,
	    sc->sc_dmap_cpbase->dm_segs[0].ds_addr);

	bus_space_write_4(sc->sc_memt, sc->sc_memh, 0x18, 6);
	bus_space_write_4(sc->sc_memt, sc->sc_memh, 0x28, 6);
	bus_space_write_4(sc->sc_memt, sc->sc_memh, 0x38, 6);

	if (pci_intr_map_msix(pa, 0, &sc->sc_ih0) != 0) {
		printf(": unable to map interrupt 0\n");
		goto destroy;
	}
	sc->sc_ihc0 = pci_intr_establish(sc->sc_pc, sc->sc_ih0, IPL_BIO,
	    vkey_intr, sc, sc->sc_dev.dv_xname);
	if (sc->sc_ihc0 == NULL) {
		printf(": unable to establish msix interrupt 0\n");
		goto destroy;
	}
	if (pci_intr_map_msix(pa, 1, &sc->sc_ih1) != 0) {
		printf(": unable to map interrupt 1\n");
		goto destroy;
	}
	sc->sc_ihc1 = pci_intr_establish(sc->sc_pc, sc->sc_ih1, IPL_BIO,
	    vkey_fail, sc, sc->sc_dev.dv_xname);
	if (sc->sc_ihc1 == NULL) {
		printf(": unable to establish msix interrupt 1\n");
		goto destroy;
	}

	return;

destroy:
	bus_dmamap_destroy(sc->sc_dmat, sc->sc_dmap_cbase);
	bus_dmamap_destroy(sc->sc_dmat, sc->sc_dmap_rbase);
	bus_dmamap_destroy(sc->sc_dmat, sc->sc_dmap_cpbase);

unmap:
	bus_space_unmap(sc->sc_memt, sc->sc_memh, sc->sc_mems);
	sc->sc_mems = 0;
}

int
vkeyopen(dev_t dev, int mode, int flags, struct proc *p)
{
	dev_t unit = minor(dev);
	struct vkey_softc *sc;

	if (unit > 0) {
		return (ENXIO);
	}

	sc = vkey_cd.cd_devs[unit];
	return (0);
}

int
vkeyclose(dev_t dev, int flag, int mode, struct proc *p)
{
	return (0);
}

struct entry *
find_entry(struct vkey_softc *sc, unsigned long cookie)
{
	struct entry *np;
	SLIST_FOREACH(np, &sc->cmds, entries) {
		if (np->cookie == cookie) {
			return np;
		}
	}
	return NULL;
}

static int
vkey_intr(void* arg)
{
	struct vkey_softc *sc = arg;
	mtx_enter(&sc->sc_mtx);
	int index = 0;

	int processed = 0;

	for (int i = 0; i < 64; i++) {
		bus_dmamap_sync(sc->sc_dmat, sc->sc_dmap_cbase, 0,
		    sc->sc_dmap_cbase->dm_segs[0].ds_len,
		    BUS_DMASYNC_PREWRITE | BUS_DMASYNC_PREREAD);
		uint8_t owner = sc->sc_cpbase[i].owner;
		bus_space_barrier(sc->sc_memt, sc->sc_memh, 0, 0x48,
		    BUS_SPACE_BARRIER_READ);

		if (owner == HOST_OWNER) {
			if (sc->sc_cpbase[i].msglen == 0 &&
			    sc->sc_cpbase[i].type == 0) {

				struct entry *np = find_entry(sc,
				    sc->sc_cpbase[i].cmdcookie);
				index = np->index;

				if (sc->sc_rbase[index].owner == HOST_OWNER) {

					np->rindex = index;
					sc->sc_rbase[index].cookie =
					    np->cookie;
					processed++;
				}
			} else {
				struct vkey_cmd_arg *arg;
				struct entry *np = find_entry(sc,
				    sc->sc_cpbase[i].cmdcookie);
				index = np->index;
				arg = np->arg;
				np->rindex = i;
				np->rtype = sc->sc_cpbase[i].type;
				np->rlen = sc->sc_cpbase[i].msglen;

				np->finished = true;
				sc->sc_rbase[index].owner = DEVICE_OWNER;
				wakeup(&sc->sc_state);
				processed++;
			}

			sc->sc_cpbase[i].owner = DEVICE_OWNER;

			bus_space_barrier(sc->sc_memt, sc->sc_memh,
			    0, 0x48, BUS_SPACE_BARRIER_WRITE);
			bus_space_write_4(sc->sc_memt, sc->sc_memh, 0x44, i);
			bus_space_barrier(sc->sc_memt, sc->sc_memh,
			    0, 0x48, BUS_SPACE_BARRIER_WRITE);

		} else if (processed > 0) {
			break;
		}

		bus_dmamap_sync(sc->sc_dmat, sc->sc_dmap_cbase, 0,
		    sc->sc_dmap_cbase->dm_segs[0].ds_len,
		    BUS_DMASYNC_POSTWRITE | BUS_DMASYNC_POSTREAD);

		bus_space_barrier(sc->sc_memt, sc->sc_memh, 0, 0x48,
		    BUS_SPACE_BARRIER_WRITE);
	}
	mtx_leave(&sc->sc_mtx);
	return (1);
}

static int
vkey_fail(void* arg)
{
	struct vkey_softc *sc = arg;
	mtx_enter(&sc->sc_mtx);
	printf("vkey failed\n");
	int flag = bus_space_read_4(sc->sc_memt, sc->sc_memh, 0x08);
	if (flag & (1 << 0)) printf("FLTB\n");
	if (flag & (1 << 1)) printf("FLTR\n");
	if (flag & (1 << 2)) printf("DROP\n");
	if (flag & (1 << 3)) printf("OVF\n");
	if (flag & (1 << 4)) printf("SEQ\n");
	if (flag & (1 << 5)) printf("HWERR\n");

	struct entry *np;
	SLIST_FOREACH(np, &sc->cmds, entries) {
		np->finished = true;
		np->error = EIO;
	}

	wakeup(&sc->sc_state);
	mtx_leave(&sc->sc_mtx);

	return (1);
}

int
vkey_cmd(struct proc *p, struct vkey_softc *sc, struct vkey_cmd_arg *va)
{
	bus_dmamap_t cmdmap, rmap;
	bus_dma_segment_t rsegs[1], csegs[1];
	int rnsegs, cnsegs;

	caddr_t cmd_p, r_p;
	struct uio uio_cmd;
	struct entry *cmd;
	unsigned long cookie;
	int index;
	int error = 0;

	int t_len = va->vkey_in[0].iov_len + va->vkey_in[1].iov_len +
	    va->vkey_in[2].iov_len + va->vkey_in[3].iov_len;

	if (t_len > 16384) {
		return ENOMEM;
	}

	r_p = vkey_dma_allocmem2(sc, 5*PAGE_SIZE, 0, &rmap, rsegs, &rnsegs);

	if (r_p == NULL) {
		return ENOMEM;
	}

	cmd = malloc(sizeof(struct entry), M_TEMP, M_WAITOK);

	if (t_len > 0) {
		cmd_p = vkey_dma_allocmem2(sc, t_len, 0, &cmdmap, csegs,
		    &cnsegs);
		if (cmd_p == NULL) {
			error = ENOMEM;
			goto free;
		}

		uio_cmd.uio_iov = va->vkey_in;
		uio_cmd.uio_iovcnt = nitems(va->vkey_in);

		uio_cmd.uio_offset = 0;
		uio_cmd.uio_resid = t_len;
		uio_cmd.uio_rw = UIO_WRITE;
		uio_cmd.uio_segflg = UIO_USERSPACE;
		uio_cmd.uio_procp = p;

		int error = uiomove(cmd_p, t_len, &uio_cmd);

		if (error) {
			error = EIO;
			goto free;
		}
	}

	mtx_enter(&sc->sc_mtx);
	index = sc->index;
	sc->index = (sc->index + 1) % 64;

	arc4random_buf(&cookie, 8);

	cmd->cookie = cookie;
	cmd->rmap = rmap;
	cmd->index = index;

	bus_dmamap_sync(sc->sc_dmat, sc->sc_dmap_rbase, 0,
	    sc->sc_dmap_rbase->dm_segs[0].ds_len,
	    BUS_DMASYNC_PREWRITE | BUS_DMASYNC_PREREAD);

	sc->sc_rbase[index].length1 = 5*PAGE_SIZE;
	sc->sc_rbase[index].cookie = cookie;
	sc->sc_rbase[index].pointer1 = (void *)rmap->dm_segs[0].ds_addr;

	bus_space_barrier(sc->sc_memt, sc->sc_memh, 0, 0x48,
	    BUS_SPACE_BARRIER_WRITE);
	sc->sc_rbase[index].owner = DEVICE_OWNER;
	bus_space_barrier(sc->sc_memt, sc->sc_memh, 0, 0x48,
	    BUS_SPACE_BARRIER_WRITE);

	bus_space_write_4(sc->sc_memt, sc->sc_memh, 0x40, (index | (1 << 31)));

	bus_dmamap_sync(sc->sc_dmat, sc->sc_dmap_rbase, 0,
	    sc->sc_dmap_rbase->dm_segs[0].ds_len,
	    BUS_DMASYNC_POSTWRITE | BUS_DMASYNC_POSTREAD);

	bus_dmamap_sync(sc->sc_dmat, sc->sc_dmap_cbase, 0,
	    sc->sc_dmap_cbase->dm_segs[0].ds_len,
	    BUS_DMASYNC_PREWRITE | BUS_DMASYNC_PREREAD);

	if (t_len > 0) {
		sc->sc_cbase[index].length1 = t_len;
		sc->sc_cbase[index].pointer1 = (void *)cmdmap->dm_segs[0].
		    ds_addr;
		cmd->length = t_len;
		cmd->pointer = cmd_p;
	}

	sc->sc_cbase[index].type = va->vkey_cmd;
	sc->sc_cbase[index].cookie = cookie;

	bus_space_barrier(sc->sc_memt, sc->sc_memh, 0, 0x48,
	    BUS_SPACE_BARRIER_WRITE);
	sc->sc_cbase[index].owner = DEVICE_OWNER;
	bus_space_barrier(sc->sc_memt, sc->sc_memh, 0, 0x48,
	    BUS_SPACE_BARRIER_WRITE);

	bus_space_write_4(sc->sc_memt, sc->sc_memh, 0x40, index);
	bus_dmamap_sync(sc->sc_dmat, sc->sc_dmap_cbase, 0,
	    sc->sc_dmap_cbase->dm_segs[0].ds_len,
	    BUS_DMASYNC_POSTWRITE | BUS_DMASYNC_POSTREAD);

	cmd->finished = false;
	cmd->arg = va;
	cmd->rpointer = r_p;
	cmd->error = 0;

	SLIST_INSERT_HEAD(&sc->cmds, cmd, entries);

	do {
		int err = msleep_nsec(&sc->sc_state, &sc->sc_mtx,
		    PRIBIO, "vkeywait", INFSLP);
		if (error) {
			error = err;
			goto free;
		}
	} while (cmd->finished != true);

	int rtotal = va->vkey_out[0].iov_len + va->vkey_out[1].iov_len +
	    va->vkey_out[2].iov_len + va->vkey_out[3].iov_len;

	error = cmd->error;
	if (error) {
		goto free;
	}

	if (rtotal > cmd->rlen || va->vkey_flags) {
		int remainder = cmd->rlen;
		int previous = 0;

		va->vkey_reply = cmd->rtype;
		va->vkey_rlen = cmd->rlen;

		if (remainder <= va->vkey_out[0].iov_len && remainder > 0) {
			copyout(r_p + previous, va->vkey_out[0].iov_base,
			    remainder);
			remainder = 0;

		} else if (remainder > 0) {
			copyout(r_p + previous, va->vkey_out[0].iov_base,
			    va->vkey_out[0].iov_len);
			remainder -= va->vkey_out[0].iov_len;
			previous += va->vkey_out[0].iov_len;
		}

		if (remainder <= va->vkey_out[1].iov_len && remainder > 0) {
			copyout(r_p + previous, va->vkey_out[1].iov_base,
			    remainder);
			remainder = 0;

		} else if (remainder > 0) {
			copyout(r_p + previous, va->vkey_out[1].iov_base,
			    va->vkey_out[1].iov_len);
			remainder -= va->vkey_out[1].iov_len;
			previous += va->vkey_out[1].iov_len;
		}

		if (remainder <= va->vkey_out[2].iov_len && remainder > 0) {
			copyout(r_p + previous, va->vkey_out[2].iov_base,
			    remainder);
			remainder = 0;

		} else if (remainder > 0) {
			copyout(r_p + previous, va->vkey_out[2].iov_base,
			    va->vkey_out[2].iov_len);
			remainder -= va->vkey_out[2].iov_len;
			previous += va->vkey_out[2].iov_len;
		}

		if (remainder <= va->vkey_out[3].iov_len && remainder > 0) {
			copyout(r_p + previous, va->vkey_out[3].iov_base,
			    remainder);
			remainder = 0;

		} else if (remainder > 0) {
			copyout(r_p + previous, va->vkey_out[3].iov_base,
			    va->vkey_out[3].iov_len);
			remainder -= va->vkey_out[3].iov_len;
			previous += va->vkey_out[3].iov_len;
		}
	} else if (cmd->rlen == 0) {
		va->vkey_reply = cmd->rtype;
		va->vkey_rlen = cmd->rlen;
	} else {
		error = EFBIG;
	}

free:
	SLIST_REMOVE(&sc->cmds, cmd, entry, entries);
	free(cmd, M_WAITOK, sizeof(struct entry));

	if (t_len > 0) {
		bus_dmamem_unmap(sc->sc_dmat, cmd_p, t_len);
		bus_dmamap_unload(sc->sc_dmat, cmdmap);
		bus_dmamem_free(sc->sc_dmat, csegs, cnsegs);
		bus_dmamap_destroy(sc->sc_dmat, cmdmap);
	}

	bus_dmamem_unmap(sc->sc_dmat, r_p, 5*PAGE_SIZE);
	bus_dmamap_unload(sc->sc_dmat, rmap);
	bus_dmamem_free(sc->sc_dmat, rsegs, rnsegs);
	bus_dmamap_destroy(sc->sc_dmat, rmap);

	mtx_leave(&sc->sc_mtx);
	return error;
}


int
vkeyioctl(dev_t dev, u_long cmd, caddr_t data, int flag, struct proc *p)
{
	dev_t unit = minor(dev);
	struct vkey_softc *sc;

	if (unit > 0) {
		return (ENXIO);
	}
	sc = vkey_cd.cd_devs[unit];
	int error = 0;

	switch (cmd) {
	case VKEYIOC_GET_INFO:
		mtx_enter(&sc->sc_mtx);

		bus_space_barrier(sc->sc_memt, sc->sc_memh, 0, 0x48,
		    BUS_SPACE_BARRIER_READ);
		int major = bus_space_read_4(sc->sc_memt, sc->sc_memh, 0x00);
		int minor = bus_space_read_4(sc->sc_memt, sc->sc_memh, 0x04);

		struct vkey_info_arg *info = (struct vkey_info_arg *)data;
		info->vkey_major = major;
		info->vkey_minor = minor;

		mtx_leave(&sc->sc_mtx);

		break;
	case VKEYIOC_CMD:
		error = vkey_cmd(p, sc, (struct vkey_cmd_arg *)data);
		break;
	default:
		error = ENOTTY;
		break;
	}

	return (error);
}