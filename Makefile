all: seewaves

seewaves::
	cd src;make
	cp src/seewaves bin/.

clean::
	cd src;make clean

