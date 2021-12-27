#include <assert.h>
#include <tyndall/ipc/ipc.h>
#include <tyndall/meta/macro.h>
#include <thread>
#include <stdlib.h>

struct my_struct
{
  long a;
  double b;
  char c;
  unsigned long d;
  bool operator==(const my_struct&) const = default;

  my_struct& operator++()
  {
    ++a;
    ++b;
    ++c;
    ++d;
    return *this;
  }

  bool has_consistent_even_ness() const
  {
    int first_bit = (a & 1);

    return (((int)b & 1) == first_bit)
      && ((c & 1) == first_bit)
      && ((d & 1) == first_bit);
  }
};

#define check(cond) do { if (!(cond)){ ipc_cleanup(); printf( __FILE__ ":" M_STRINGIFY(__LINE__) " " "Assertion failed: " #cond "\n"); exit(1); }} while(0)

int main()
{
  ipc_cleanup();

  const int n_threads = 10;
  std::thread threads[n_threads];

  const int n_ids = 10 * n_threads;
  const int id_length = 15;
  char ids[n_ids][id_length + 1];

  const char id_char_begin = 97;
  const char id_char_end = 123;
  //printf("begin, end-1: %c %c\n", id_char_begin, id_char_end-1);

  for (int i=0; i < n_ids; ++i)
  {
    for (int j=0; j < id_length; ++j)
      ids[i][j] = id_char_begin + rand() % (id_char_end - id_char_begin);
    ids[i][id_length] = '\0';
    //printf("id %d: %s\n", i, &ids[i][0]);
  }

  const int max_ids_per_thread = (n_ids / n_threads) * 5;
  static_assert(max_ids_per_thread < n_ids);
  //printf("max_ids_per_thread: %d\n", max_ids_per_thread);
  // array to divide the ids between threads
  int ids_per_thread[n_threads][max_ids_per_thread];
  std::fill(&ids_per_thread[0][0], &ids_per_thread[n_threads][max_ids_per_thread], -1);

  int writing_thread_for_id[n_ids] = {0};

  static_assert(n_ids >= n_threads);
  for (int i=0; i < n_threads; ++i)
  {
    // give ids to the thread
    const int thread_ids = 1 + rand() % (max_ids_per_thread - 1);
    for (int j=0; j < thread_ids; ++j)
    {
      ids_per_thread[i][j] = rand() % n_ids;

      // dont insert duplicate ids
      bool duplicate;
      do {
        duplicate = false;
        for (int k = 0; k < j; ++k)
        {
          if (ids_per_thread[i][k] == ids_per_thread[i][j])
          {
            duplicate = true;
            ids_per_thread[i][j] = (ids_per_thread[i][j] + 1) % n_ids;
          }
        }
      } while (duplicate);

      // register thread as writer if there is no other writer of the id
      if (writing_thread_for_id[ids_per_thread[i][j]] == 0)
        writing_thread_for_id[ids_per_thread[i][j]] = i;
    }

    // initialize thread
    threads[i] = std::thread{[thread_index = i, ids, n_ids, ids_per_thread, thread_ids, writing_thread_for_id]()
      {
        int n_writers = 0, n_readers = 0;
        ipc_rtid_writer<my_struct> writers[thread_ids];
        ipc_rtid_reader<my_struct> readers[thread_ids];

        // initialise writers and readers
        for (int i=0; i < thread_ids; ++i)
        {
          const int id_index = ids_per_thread[thread_index][i];
          const char* id = &ids[id_index][0];
          if (writing_thread_for_id[id_index] == thread_index)
            writers[n_writers++].init(id);
          else
            readers[n_readers++].init(id);
        }

        my_struct write_entry = {0};

        const int n_iterations = 100;
        for (int i=0; i<n_iterations; ++i)
        {
          // write
          for (int i=0; i < n_writers; ++i)
            writers[i].write(++write_entry);

          // read
          for (int i=0; i < n_readers; ++i)
          {
            my_struct read_entry;
            readers[i].read(read_entry);

            printf("thread %d: checking %ld\n", read_entry.a);

            check(read_entry.has_consistent_even_ness());
          }
        }
      }
    };
  }

  for (auto& thread : threads)
    thread.join();

  ipc_cleanup();
}
