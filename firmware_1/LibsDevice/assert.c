

//#define assert_param(expr) ((void)0)

void assert_failed(uint8_t* file, uint32_t line)
{
   (void)file;
   (void)line;
   while (1);
}


/*
void memcpy(void *dest, void *src, int n)
{
   char *csrc = (char *)src;
   char *cdest = (char *)dest;
  
   for (int i=0; i<n; i++)
   {
    cdest[i] = csrc[i];
   }
}
*/