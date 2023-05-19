1. partitioning. ctest wb_dma still need to be checked


5/16/2023
1. revise partition parameters, handle standalone node clusters.
2. for 1 thread, 1 partition, ctest mostly went through, for those who didn't, there was segmentation fault on mt-kahypar
   I tried it with mt-kahypar example, still got segmentation fault sometimes. Might think about switch to other partitioner
3. need to check multiple threads and multiple partitions


5/17/2023
1. BUGs in multiple partitions solved. Before I used pin state to judge if a pin from a partition should do ftask or btask, this will cause BUG if multiple partitions have the same pin, then the pin from the second partition will skip the ftask and IMMEDIATELY to btask, which is wrong.
  So I seperate the original toplogically sorted partitions into 2 parts, one for ftask and one for btask, but will it have a big impact on performance?
2. I need to add dependency for ftask partitions and btask partitions

5/17/2023
1. solved bugs for multi=threads by seperate the original topologicall sorted partitions into 2 parts
2. ctest run through EXCEPT des_perf
3. check runtime, cluster takes a huge amount of time. wb_dma runs 7.0 s and cluster takes 5.0 s, and quick sort in cluster takes most of the time 
4. make thread pool of partitioner into constructor, use multi-thread and there is no bug
5. use bucket list std::vector<std::vetcor<Pin*>> to avoid sort

5/18/2023
1. remove sorting.
2. ctest passed and takes 37 s (original taskflow task 20 s)

5/19/2023
1. baseline implementation done. partition the whole taskflow graph
