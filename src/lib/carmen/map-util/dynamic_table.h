 /*********************************************************
 *
 * This source code is part of the Carnegie Mellon Robot
 * Navigation Toolkit (CARMEN)
 *
 * CARMEN Copyright (c) 2002 Michael Montemerlo, Nicholas
 * Roy, Sebastian Thrun, Dirk Haehnel, Cyrill Stachniss,
 * and Jared Glover
 *
 * CARMEN is free software; you can redistribute it and/or 
 * modify it under the terms of the GNU General Public 
 * License as published by the Free Software Foundation; 
 * either version 2 of the License, or (at your option)
 * any later version.
 *
 * CARMEN is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied 
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE.  See the GNU General Public License for more 
 * details.
 *
 * You should have received a copy of the GNU General 
 * Public License along with CARMEN; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place, 
 * Suite 330, Boston, MA  02111-1307 USA
 *
 ********************************************************/

#ifndef DYNAMICTABLE_H
#define DYNAMICTABLE_H

#include <stdlib.h>

/** a dynnamic array of pointer to elements of type T 
  * witch grows automaticly.
  *@author Cyrill Stachniss
  */
template<class T>
class DynamicTable {
public: 
  /** O(1) */
  DynamicTable(int s=10);

  /** O(n)  */
  DynamicTable(DynamicTable<T>& tab, int desired_size=-1);

  /** O(1) if autodelete==false, else O(n) */
  ~DynamicTable();

  /** sets the autodelete variable -> see autoDelete in this class , O(1)*/
  void setAutoDelete(bool del);

  /** O(1) */
  int numberOfElements() const ;
  /** O(1), does the same as numberOfElements */
  int num() const;

  /** O(1), allocated memory (size of array) */
  int allocated() const;

  /** O(n) */
  bool contains(T* element) const;

  /** O(1) */
  T* getElement(int i);
  /** O(1) */
  T* getFirst();
  /** O(1) */
  T* getLast();
  /** O(1) */
  T** getElements();

  /** O(1) */
  const T* getElement(int i) const ;
  /** O(1) */
  const T* getFirst() const ;
  /** O(1) */
  const T* getLast() const ;
  /** O(1) */
  const T** getElements() const ;

  
  /** O(1) */
  void add(T* element);
  /** O(n + nOther) */
  void add(DynamicTable<T>& otherTab);

  /** O(1) */
  T* replace(int idx, T* elem);
  /** O(1) */
  void swap(int idx1, int idx2);


  /**  O(1), BUT DISTRUBS THE TABLE, CALL CONSOLIDATE AFTER THIS!!!!!!!!*/
  void remove_unconsolidated(int i);
  /*** O(n) **/
  void consolidate();


  /** O(1) */
  void removeLast();
  /** Destroys the order ,O(1) */
  void remove(int i);
  /** O(n) */
  void remove(T* element);
  /** O(n) */
  void removeByValue(T* element) { remove(element); }
  /** maintais the order, O(n) */
  void removeInOrder(int i);

  /** O(1) if autodelete==false, else O(n) */
  void clear();

  /** O(n) */
  void reverse();

  /** O(n + nOther) */
  DynamicTable<T>* getIntersection(DynamicTable<T>* other);


protected:
  /** Array von Zeigern auf die einzelnen fields-Elemente */
  T** fields;
  int size;
  int nextField;
		
  /** delete the elements in the table during destruction */
  bool autoDelete;
};

template<class T>
DynamicTable<T>::DynamicTable(int s) {
  size=s;
  setAutoDelete(false);

  if ( s < 1) {
    size = 6;
    fprintf(stderr, "%s at line %d : error in table, initial size s=%i! "
      "setting size=%i\n", __FILE__,__LINE__,s,size);
  }

  // size should be even
  if ((size % 2) == 1)
    size++;

  nextField = 0;
  fields = new T*[size]; /* check_alloc checked */
  if (fields == NULL) {
    fprintf(stderr,"out of memory! %s at line %d, size=%d\n", __FILE__,
      __LINE__, size);
    exit(0);
  }
  fields[0] = NULL;
}



template<class T>
DynamicTable<T>::DynamicTable(DynamicTable<T>& tab, int desired_size) {

  setAutoDelete(tab .autoDelete);

  if (desired_size > tab.nextField) {
    size = desired_size;
    nextField = tab.nextField;
  }
  else {
    size = tab.size;
    nextField = tab.nextField;
  }

  fields = new T*[size]; /* check_alloc checked */
  if (fields == NULL) {
    fprintf(stderr,"out of memory! %s at line %d, size=%d\n", __FILE__,__LINE__,
size);
    exit(0);
  }

  for (int i=0; i < nextField; i++) {
    fields[i] = tab.fields[i];
  }

  if (nextField < size)
    fields[nextField] = NULL;
}



template<class T>
DynamicTable<T>::~DynamicTable() {
  if (autoDelete)    {
    for(int n=0; n < numberOfElements(); n++) {
      if (fields[n] != NULL) {
  delete fields[n];
  fields[n] = NULL;
      }
    }
  }

  delete [] fields;
}


template<class T>
void DynamicTable<T>::add(DynamicTable<T>& otherTab) {

  int needMore =  otherTab.numberOfElements() -  (size - nextField) + 1;

  if (needMore > 0) {
    T** fields2 = new T*[2*size+needMore]; /* check_alloc checked */
    if (fields2 == NULL) {
      fprintf(stderr,"out of memory! %s at line %d, size=%d\n",
        __FILE__,__LINE__, 2*size+needMore);
      exit(0);
    }

    for (int i = 0; i < nextField; i++)
      fields2[i] = fields[i];
    size = 2*size+needMore;
    delete [] fields;
    fields = fields2;
  }

  int othersize = otherTab.numberOfElements();
  for (int i = 0; i < othersize; i++) {
    fields[nextField++] = otherTab.fields[i];
  }
  fields[nextField] = NULL;
}

/** Fuegt ein Feld an */
template<class T>
void DynamicTable<T>::add(T* element) {
  if (nextField == size)    {
    T** fields2 = new T*[2*size];/* check_alloc checked */
    if (fields2 == NULL) {
      fprintf(stderr,"out of memory! %s at line %d, size=%d\n",
        __FILE__,__LINE__, 2*size);
      exit(0);
    }
    for (int i = 0; i < nextField; i++)
      fields2[i] = fields[i];
    size *= 2;
    delete [] fields;
    fields = fields2;
  }

  fields[nextField++] = element;
  fields[nextField] = NULL;
}

/** Liefert das Field-Array selbst */
template<class T>
T** DynamicTable<T>::getElements() {
  return fields;
}


/** Liefert Zeiger auf das i-te Element */
template<class T>
T* DynamicTable<T>::getElement(int i) {
  if (i >= nextField)
    return NULL;
  else
    return fields[i];
}

/** Liefert Zeiger auf das 1-te Element */
template<class T>
T* DynamicTable<T>::getFirst() {
  if (nextField <= 0)
    return NULL;
  else
    return fields[0];
}


/** Liefert Zeiger auf das letzte Element */
template<class T>
T* DynamicTable<T>::getLast()  {
  if (nextField <= 0)
    return NULL;
  else
    return fields[nextField-1];
}


/** Liefert das Field-Array selbst */
template<class T>
const T** DynamicTable<T>::getElements() const {
  return fields;
}


/** Liefert Zeiger auf das i-te Element */
template<class T>
const T* DynamicTable<T>::getElement(int i) const {
  if (i >= nextField)
    return NULL;
  else
    return fields[i];
}

/** Liefert Zeiger auf das 1-te Element */
template<class T>
const T* DynamicTable<T>::getFirst() const {
  if (nextField <= 0)
    return NULL;
  else
    return fields[0];
}


/** Liefert Zeiger auf das letzte Element */
template<class T>
const T* DynamicTable<T>::getLast() const {
  if (nextField <= 0)
    return NULL;
  else
    return fields[nextField-1];
}


template<class T>
int DynamicTable<T>::numberOfElements() const {
  return nextField;
}

template<class T>
int DynamicTable<T>::num() const {
  return nextField;
}

template<class T>
int DynamicTable<T>::allocated() const {
  return size;
}



template<class T>
void DynamicTable<T>::reverse() {
  if (nextField <= 1)
    return;

  T* tmp = NULL;

  for (int i=0; i < nextField / 2; i++) {
    tmp = fields[i];
    fields[i] = fields[nextField-1-i];
    fields[nextField-1-i] = tmp;
  }
}

/** sets the autodelete variable -> see autoDelete in this class */
template<class T>
void DynamicTable<T>::setAutoDelete(bool del) {
  autoDelete = del;
}

template<class T>
void DynamicTable<T>::removeLast() {

  if (nextField == 0)
    return;

  nextField--;
  if (autoDelete)
    delete fields[nextField];
  fields[nextField] = NULL;
}



template<class T>
void DynamicTable<T>::remove_unconsolidated(int i) {
  if (i < 0 || i >= nextField || nextField <= 0)
    return;
  if (autoDelete && fields[i] != NULL)
    delete fields[i];
  fields[i] = NULL;
}

template<class T>
void DynamicTable<T>::consolidate() {

  int i=0,j=0;
  while(i < nextField) {
    if (fields[i] == NULL) {
      while (fields[j] == NULL && j < nextField)
  j++;
      if (j == nextField) {
  nextField = i;
  return;
      }
      fields[i] = fields[j];
      fields[j] = NULL;
    }
    j++;
    i++;
  }

}

template<class T>
void DynamicTable<T>::remove(int i) {

  if ((i < nextField) && (nextField > 0) && (i>=0)) {
    if (autoDelete)
      delete fields[i];

    fields[i] = fields[nextField-1];
    fields[nextField-1] = NULL;
    nextField--;
  }
}

template<class T>
void DynamicTable<T>::removeInOrder(int i) {

  if ((i < nextField) && (nextField > 0) && (i>=0)) {
    if (autoDelete)
      delete fields[i];
    for (int j = i; j < nextField-1; j++)
      fields[j] = fields[j+1];
    fields[nextField-1] =  NULL;
    nextField--;
  }
}

template<class T>
void DynamicTable<T>::remove(T* element) {

  for(int i = 0; i < nextField; i++) {
    if (fields[i] == element) {
      if (autoDelete)
  delete fields[i];
      fields[i] = fields[nextField-1];
      fields[nextField-1] = NULL;
      nextField--;
    }
  }
}

template<class T>
void DynamicTable<T>::clear() {

  if (autoDelete) {
    for(int i = 0; i < nextField; i++)
      delete fields[i];
  }
  nextField = 0;
  fields[0] = NULL;
}

template<class T>
DynamicTable<T>*  DynamicTable<T>::getIntersection(DynamicTable<T>* other) {
  DynamicTable<T>* intersection = new DynamicTable<T>(size+1);
  if (intersection == NULL) {
    fprintf(stderr,"out of memory! %s at line %d\n", __FILE__,__LINE__);
    exit(0);
  }

  T** elemOther = other->getElements();

  for (int i = 0; i < other->numberOfElements(); i++) {
    int j = 0;
    bool abort = false;
    while (!abort && j < numberOfElements()) {

      if (elemOther[i] == fields[j]) {
        intersection->add(fields[j]);
        abort = true;
      }
      else
        j++;
    }
  }
  return intersection;
}

template<class T>
bool DynamicTable<T>::contains(T* element) const {

  for (int i = 0; i < numberOfElements(); i++) {
    if (element == fields[i])
      return true;
  }
  return false;
}

template<class T>
T* DynamicTable<T>::replace(int idx, T* elem) {
  T* replaced = NULL;
  if (idx >= 0 && idx < nextField) {
    replaced = fields[idx];
    fields[idx] = elem;
  }
  return replaced;
}

template<class T>
void DynamicTable<T>::swap(int idx1, int idx2) {
  if (idx1 >= 0 && idx1 < nextField && idx2 >= 0 && idx2 < nextField) {
    T* tmp = fields[idx1];
    fields[idx1] = fields[idx2];
    fields[idx2] = tmp;
  }
}

#endif
