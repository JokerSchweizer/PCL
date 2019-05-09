#include<stdio.h>
#include<stdlib.h>
#include<string.h>
#include<math.h>
 
int eage_size;
int vertax_size;
char filename_eage[200];
char filename_vertax[200];
 
int** eage_set;
char** vertax_set;
int** adjacentMatrix;
int* visitedFlag;
 
typedef struct SequenceStack
{
	int* base;
	int* top;
	int stackSize;
}SequenceStack;
 
void readEageDataFromFile();
void readVertaxDataFromFile();
void createAdjacentMatrix();
void DFS(int); 
void DFSTraverse();
void initialVisitedFlagArray();
void printVisitedVertax(int); 
void setVisitedFlag(int,int); 
int firstAdjacentVertax(int); 
int nextAdjacentVertax(int,int);
 
void initializeSequenceStack(SequenceStack*);
void pop_stack(SequenceStack*, int*);
void push_stack(SequenceStack*, int);
void print_stack(SequenceStack);
int empty_stack(SequenceStack);
void clear_stack(SequenceStack*);
void test_stack();
 
int main(int argc, char* argv[])
{
	if( argc != 5 )
	{
		printf("\tThis algorithm require 3 parameters"
				"\n\t\t1:the size of eage"
				"\n\t\t2:the filename contain eage-data"
				"\n\t\t3:the size of vertax"
				"\n\t\t4:the filename contain vertax-data");
		exit(0);
	}
	eage_size = atoi(argv[1]);
	strcat(filename_eage, argv[2]);
	vertax_size = atoi(argv[3]);
	strcat(filename_vertax, argv[4]);
	printf("eage_size : %d, vertax_size : %d, filename-eage : %s, filename-vertax : %s\n", eage_size, vertax_size, filename_eage, filename_vertax);
	readEageDataFromFile();
	readVertaxDataFromFile();
	createAdjacentMatrix();
	DFSTraverse();
	//test_stack(); 
	return 0; 
}
 

 
int loop_count;
int heap;
int innerStep = 0;
int temp;
int isRecall;
SequenceStack loop_stack;
int pop_value;
void DFS(int startVertax)
{
	setVisitedFlag(startVertax, 1);
	int nextVertax;
	push_stack(&loop_stack, startVertax);
	nextVertax = firstAdjacentVertax(startVertax);
	innerStep++;
	for( ; ; )
	{
		if( nextVertax != -1 )
		{
			if( visitedFlag[nextVertax] == 1 && nextVertax == heap && innerStep == 2 )
			{
				nextVertax = nextAdjacentVertax(startVertax, nextVertax);
				continue;
			}
			else if( visitedFlag[nextVertax] == 1 && nextVertax == heap && innerStep != 2 )
			{
				print_stack(loop_stack);
				nextVertax = nextAdjacentVertax(startVertax, nextVertax);
				continue;
			}
			else if( visitedFlag[nextVertax] == 0 )
			{
				DFS(nextVertax);
			}
			if( isRecall == 1 )
			{
				innerStep--;
				temp = nextVertax;
				nextVertax = nextAdjacentVertax(startVertax, nextVertax);
				pop_stack(&loop_stack, &pop_value);
				setVisitedFlag(temp, 0);
				isRecall = 0;
				continue;
			}
			nextVertax = nextAdjacentVertax(startVertax, nextVertax);
		}
		else if( nextVertax == -1 )
		{
			isRecall = 1;
			break;
		}
	}
}
void DFSTraverse()
{
	initialVisitedFlagArray();
	initializeSequenceStack(&loop_stack);
	int i;
	for( heap = 1; heap <= vertax_size; heap++ )
	{
		for( i = 1; i <= vertax_size; i++ )
		{
			visitedFlag[i] = 0;
		}
		/*
		printf("print the visitedFlag array: ");
		for( i = 1; i <= vertax_size; i++ )
		{
			printf("%d ", visitedFlag[i]);
		}
		printf("\n");
		*/
		if( visitedFlag[heap] == 0 )
		{
			printf("\n-------------------the loop start and end with %d----------------\n", heap);
			clear_stack(&loop_stack);
			innerStep = 0;
			//printf("isRecall : %d, findLoop : %d, hasOthers : %d\n", isRecall, findLoop, hasOthers);
			isRecall = 0;
			DFS(heap);
		}
	}
}
void initialVisitedFlagArray()
{
	visitedFlag = (int*)malloc(sizeof(int) * (vertax_size + 1));
	if( !visitedFlag )
	{
		printf("visitedFlag* malloc error");
		exit(0);
	}
	int i;
	for( i = 1; i <= vertax_size; i++ )
		visitedFlag[i] = 0;
}
void printVisitedVertax(int vertaxID)
{
	printf("visited: %d \n", vertaxID);
}
void setVisitedFlag(int vertaxID, int value)
{
	visitedFlag[vertaxID] = value;
}
int firstAdjacentVertax(int vertaxID)
{
	int i;
	for( i = 1; i <= vertax_size; i++ )
	{
		if( adjacentMatrix[vertaxID][i] == 1 )
			return i;
	}
	return -1;
}
int nextAdjacentVertax(int vertaxID, int nextVertaxID)
{
	int i;
	for( i = nextVertaxID + 1; i <= vertax_size; i++ )
	{
		if( adjacentMatrix[vertaxID][i] == 1 )
			return i;
	}
	return -1;
}
void initializeSequenceStack(SequenceStack* stack)
{
	stack->base = (int*)malloc(sizeof(int) * (vertax_size + 1));
	if( !stack->base )
	{
		printf("Sequence stack malloc error!\n");
		exit(0);
	}
	stack->top = stack->base;
	stack->stackSize = vertax_size;
}
void pop_stack(SequenceStack* stack, int* value)
{
	if( empty_stack(*stack) == 1 )
	{
		printf("stack is empty , can not to pop!\n");
		exit(0);
	}
	*value = *(--(stack->top));
}
void push_stack(SequenceStack* stack, int value)
{
	*(stack->top) = value;
	(stack->top)++;
}
int empty_stack(SequenceStack stack)
{
	return stack.top == stack.base ? 1 : 0;
}
void print_stack(SequenceStack stack)
{
	int temp = *(stack.base);
	while( stack.top != stack.base )
	{
		printf("%d->", *((stack.base)++));
	}
	printf("%d\n", temp);
}
void clear_stack(SequenceStack* stack)
{
	stack->top = stack->base;
}</span>
 
 