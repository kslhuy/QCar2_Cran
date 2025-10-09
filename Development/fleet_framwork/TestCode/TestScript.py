import  time
from threading import Thread
def square(n):
    return n ** 2/100

class SquareThread(Thread):
    def __init__(self, number, step):
        super().__init__()
        self.number = number
        self.step = step
    
    def run(self):
        for i in range(step*self.number+1, step*(self.number+1)):
            result = i ** 2/100
            results.append(result)
        

step = 1
n = 10000
numbers = range(1,n+1)
results = []
threads = []
print(numbers)
t = time.time()
for n in numbers:
    result = square(n)
    results.append(result)


print(time.time()-t)
t = time.time()

for n in numbers:
    thread = SquareThread(int(n/step),step)
    thread.start()
    threads.append(thread)



for thread in threads:
    thread.join()

print(time.time()-t)
# print(results)
# import asyncio
# import aiohttp

# async def producer(queue):
#     urls = [
#         "https://example.com",
#         "https://www.python.org",
#         "https://www.openai.com"
#     ]
#     for url in urls:
#         await queue.put(url)
#         print(f"Produced {url}")

# async def consumer(queue, session):
#     while True:
#         url = await queue.get()
#         if url is None:
#             break
#         async with session.get(url) as response:
#             content = await response.text()
#             print(f"Consumed {url}")
#         queue.task_done()

# async def main_queue():
#     queue = asyncio.Queue()
#     async with aiohttp.ClientSession() as session:
#         # 启动生产者任务
#         producer_task = asyncio.create_task(producer(queue))
        
#         # 启动多个消费者任务
#         consumer_tasks = [asyncio.create_task(consumer(queue, session)) for _ in range(3)]

#         await producer_task
#         await queue.join()  # 等待所有任务完成

#         # 停止消费者
#         for _ in range(3):
#             await queue.put(None)
#         await asyncio.gather(*consumer_tasks)

# asyncio.run(main_queue())