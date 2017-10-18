#Standard boilerplate
prefix = /usr/local
exec_prefix = ${prefix}
mandir = ${datarootdir}/man
includedir = ${prefix}/include
datarootdir = ${prefix}/share

hdr = $(DESTDIR)/$(includedir)/TooN

all:

install:
	mkdir -p $(hdr)
	cp *.h $(hdr)
	cp -r optimization $(hdr)/
	cp -r internal $(hdr)/

internal/make_vector.hh:  make_make_vector.awk
	awk -f make_make_vector.awk > $@

internal/builtin_typeof.h:make_typeof.awk
	awk -f make_typeof.awk > $@

clean:
	rm -rf html

docs:
	doxygen 
