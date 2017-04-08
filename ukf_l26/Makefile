all: comp run

clean:
	rm -rf ./*.exe
comp: clean
	g++  -I  ~/workspace/Eigen/ -o ukf_l26.exe main.cpp ukf.cpp

run: comp
	./ukf_l26.exe
