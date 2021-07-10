SCENARIO := Default

all: clean runsim vec2sql ipynb

clean:
	rm -f results/*
	rm -f analysis/analysis.html

runsim: run00 run01 run02 run03 run04 run05 run06 run07 run08 run09 run10 run11

.PHONY: run00
run00:
	./run -u Cmdenv -c $(SCENARIO) -r 0

.PHONY: run01
run01:
	./run -u Cmdenv -c $(SCENARIO) -r 1

.PHONY: run02
run02:
	./run -u Cmdenv -c $(SCENARIO) -r 2

.PHONY: run03
run03:
	./run -u Cmdenv -c $(SCENARIO) -r 3

.PHONY: run04
run04:
	./run -u Cmdenv -c $(SCENARIO) -r 4

.PHONY: run05
run05:
	./run -u Cmdenv -c $(SCENARIO) -r 5

.PHONY: run06
run06:
	./run -u Cmdenv -c $(SCENARIO) -r 6

.PHONY: run07
run07:
	./run -u Cmdenv -c $(SCENARIO) -r 7

.PHONY: run08
run08:
	./run -u Cmdenv -c $(SCENARIO) -r 8

.PHONY: run09
run09:
	./run -u Cmdenv -c $(SCENARIO) -r 9

.PHONY: run10
run10:
	./run -u Cmdenv -c $(SCENARIO) -r 10

.PHONY: run11
run11:
	./run -u Cmdenv -c $(SCENARIO) -r 11

.PHONY: vec2sql
vec2sql:
	/data/src/vec2sql/vec2sql.py results/$(SCENARIO)_*.vec results/$(SCENARIO).db	

ipynb:
	SCENARIO=$(SCENARIO) jupyter nbconvert --to html --execute --ExecutePreprocessor.timeout=-1 analysis/analysis.ipynb
