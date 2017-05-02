#ifndef FIFO_BUFFER_INCLUDE
#define FIFO_BUFFER_INCLUDE

#define FIFO_LENGTH 1024
//#define STORE_FAULT

struct FifoBuffer {
#ifdef STORE_FAULT
	int32_t data[FIFO_LENGTH][3];
#else
	int32_t data[FIFO_LENGTH][2];
#endif
	int first;
	int last;
	int count;
};
void fifo_init(struct FifoBuffer* bf) {
	bf->first = 0;
	bf->last = FIFO_LENGTH-1;
	bf->count = 0;
}
int fifo_numItems(struct FifoBuffer* bf) {
	return bf->count;
}
bool fifo_isEmpty(struct FifoBuffer* bf) {
	return (bf->count <= 0);	
}
bool fifo_isFull(struct FifoBuffer* bf) {
	return (bf->count >= FIFO_LENGTH);
}
bool fifo_push(struct FifoBuffer* bf, int32_t s1, int32_t s2, int32_t s3) {
	if (fifo_isFull(bf)) return false;
	bf->last = (bf->last + 1) % FIFO_LENGTH;
	bf->data[bf->last][0] = s1;
	bf->data[bf->last][1] = s2;
#ifdef STORE_FAULT
	bf->data[bf->last][2] = s3;
#endif
	bf->count = bf->count + 1;
	return true;
}
bool fifo_pop(struct FifoBuffer* bf, int32_t* s1, int32_t* s2, int32_t* s3) {
	if (fifo_isEmpty(bf)) return false;
	*s1 = bf->data[bf->first][0];
	*s2 = bf->data[bf->first][1];
#ifdef STORE_FAULT
	*s3 = bf->data[bf->first][2];
#endif
	bf->first = (bf->first + 1) % FIFO_LENGTH;
	bf->count = bf->count - 1;
	return true;
}
bool fifo_peek(struct FifoBuffer* bf, int offset, int32_t* s1, int32_t* s2, int32_t* s3) {
	if (fifo_numItems(bf) <= offset) return false;	
	int pos =  (bf->first + offset) % FIFO_LENGTH;
	*s1 = bf->data[pos][0];
	*s2 = bf->data[pos][1];
#ifdef STORE_FAULT
	*s3 = bf->data[pos][2];
#endif
	return true;
}

#endif