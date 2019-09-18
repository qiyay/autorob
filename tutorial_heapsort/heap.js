/*|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\

    Heap Sort Stencil | JavaScript support functions

    Quick JavaScript Code-by-Example Tutorial 
     
    @author ohseejay / https://github.com/ohseejay
                     / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Michigan Honor License 

|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/*/


// create empty object 
minheaper = {}; 

// define insert function for min binary heap
function minheap_insert(heap, new_element) {

    // STENCIL: implement your min binary heap insert operation

    heap.push(new_element)
    var now = heap.length - 1;
    while(now > 0){
        parent = Math.floor((now - 1) / 2)
      if (heap[parent] > heap[now]){
        var temp = heap[now];
        heap[now] = heap[parent];
        heap[parent] = temp;
        now = parent;
      }
      else{
        break
      }
    }

}

// assign insert function within minheaper object
minheaper.insert = minheap_insert;
/* Note: because the minheap_insert function is an object, we can assign 
      a reference to the function within the minheap object, which can be called
      as minheap.insert
*/

// define extract function for min binary heap
function minheap_extract(heap) {

    // STENCIL: implement your min binary heap extract operation
    // if(heap.length == 1){
    //   return heap.pop()
    // }

    // a = heap[0];
    // heap[0] = heap.pop();

    // l = 0;
    var len = heap.length;
    var a = heap[0];
    heap[0] = heap[len - 1];
    heap.pop();
    len -= 1;
    var now = 0;
    var newidx
    while(now < heap.length) {
        
        var lchild = now * 2 + 1;
        var rchild = now * 2 + 2;

        if(rchild <= len - 1){
            if(heap[now] > heap[lchild] || heap[now] > heap[rchild]){
                if(heap[lchild] < heap[rchild]) newidx = lchild;
                else newidx = rchild;
            }
            else break
        }

        else if(lchild == len - 1){
            if(heap[lchild] < heap[now]) newidx = lchild
            else break
        }

        else break
        var temp = heap[newidx];
        heap[newidx] = heap[now];
        heap[now] = temp;
        now = newidx;


        // if ((heap[fi] == undefined) && (heap[si] == undefined)){
        //     break
        // }
        // else if ((heap[l] > heap[fi]) && (heap[si] == undefined)) {
        //     temp = heap[l];
        //     heap[l] = heap [fi];
        //     heap[fi] = temp;
        //     l = fi;
        // }
        // else if ((heap[l] > heap[si]) && (heap[fi] == undefined)) {
        //     temp = heap[l];
        //     heap[l] = heap [si];
        //     heap[si] = temp;
        //     l = si;
        // }

        // else if ((heap[l] > heap[fi]) && (heap[fi] < heap[si]) && (heap[fi] != undefined) && (heap[si] != undefined)){
        //     temp = heap[l];
        //     heap[l] = heap [fi];
        //     heap[fi] = temp;
        //     l = fi;;
        // }
        // else if ((heap[l] > heap[si]) && (heap[si] < heap[fi]) && (heap[fi] != undefined) && (heap[si] != undefined)){
        //     temp = heap[l];
        //     heap[l] = heap [si];
        //     heap[si] = temp;
        //     l = si;
        // }

        // else{
        //     break
        // }


    }
    return a;
}

// assign extract function within minheaper object

    // STENCIL: ensure extract method is within minheaper object
minheaper.extract = minheap_extract;





