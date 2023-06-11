import numpy




def random_xorshift64star(g_random_state):
    
    g_random_state ^= g_random_state >> 12
    g_random_state ^= g_random_state << 25
    g_random_state ^= g_random_state >> 27
    g_random_state  = g_random_state%(1<<64)

    return g_random_state #, (g_random_state >> 32)*0x2545F4914F6CDD1D


if __name__ == "__main__":
    g_random_state = 1

    for i in range(100):
        g_random_state = random_xorshift64star(g_random_state)

        print(g_random_state%256)