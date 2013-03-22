
#include <ntk/utils/stl.h>

#include <set>
#include <iostream>
#include <iterator>

struct Predicate
{
  bool operator() (int a, int b) const
  { return 3*a == b || b*3==a; }
};

bool test_keep_maximal()
{
  std::set<int> s;
  s.insert(2);
  s.insert(8);
  s.insert(4);
  s.insert(7);
  s.insert(3);
  s.insert(9);
  s.insert(10);
  s.insert(12);
  
  std::set<int> output;
  Predicate pred;
  ntk::keep_maximal_among_sorted_objects(std::inserter(output, output.begin()), 
                                         s.begin(), s.end(), pred);
  
  if (output.size() != s.size() - 2) return false;
  if (output.find(3) != output.end()) return false;
  if (output.find(4) != output.end()) return false;
  return true;
}

int main()
{
  bool ok = true;
  ok &= test_keep_maximal();
  if (!ok) return 1;
  return 0;
}
