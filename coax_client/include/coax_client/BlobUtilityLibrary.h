/**
 * @file: BlobUtilityLibrary.h
 * @date: 2/17/2011
 * @auther: Aaron Parker
 */

#ifndef BLOB_UTILITY_LIBRARY_H
#define BLOB_UTILITY_LIBRARY_H

using namespace std;

#include <coax_client/CoaxClientConst.h>
#include <cmvision/Blobs.h>
#include <ros/ros.h>
#include <coax_client/BlobSequences.h>
#include <string.h>

bool isInList(const unsigned int &i, std::list<unsigned int> &list);
int combineBlobs(const unsigned int &b1, const unsigned int &b2, cmvision::Blobs &blobs);
bool blobsAreAdjacent(const unsigned int &b1, const unsigned int &b2, const cmvision::Blobs &blobs);
bool blobsAreSameColor(const unsigned int &b1, const unsigned int &b2, const cmvision::Blobs &blobs);
int findSimilarAdjacentBlob(const unsigned int &b1, cmvision::Blobs &blobs);
float blobAngle(const unsigned int &b1, const unsigned int &b2, const cmvision::Blobs &blobs);
int findAdjacentBlobs(const unsigned int &b1, const cmvision::Blobs &blobs, std::vector<unsigned int> &adj_blobs);
int findBlobCluster(const unsigned int &b1, const cmvision::Blobs &blobs, std::vector<unsigned int> &blob_cluster);
void deleteBlob(const unsigned int &b1, cmvision::Blobs &blobs);
void removeDuplicates(std::vector<unsigned int>& vec);
void orderCluster(std::vector<unsigned int>& cluster,cmvision::Blobs blobs);
cmvision::Blob& getBlob(const unsigned int &blob_num, cmvision::Blobs &blobs);
void printBinary(char n);
char getColorID(unsigned int blob_num,cmvision::Blobs& blobs);
void blobSequenceFromCluster(coax_client::BlobSequence &blob_sequence, std::vector<unsigned int> &cluster, cmvision::Blobs blobs);
int findAllBlobClusters(cmvision::Blobs blobs, std::vector<std::vector<unsigned int> > &blob_clusters);

bool isInList(const unsigned int &i, std::vector<unsigned int> &list)
{
    for(std::vector<unsigned int>::iterator pos = list.begin();pos != list.end(); ++pos)
    {
        if(*pos == i) return true;
    }
    return false;
}
 
int combineBlobs(const unsigned int &b1, const unsigned int &b2, cmvision::Blobs &blobs)
{
    if(b1==b2 || !blobsAreSameColor(b1,b2,blobs)) return -1;
    unsigned int max = b2;
    unsigned int min = b1;
    if(b1>b2) 
    {
        max = b1;
        min = b2;
    }
    blobs.blobs[min].top = (blobs.blobs[min].top<blobs.blobs[max].top) ? blobs.blobs[min].top : blobs.blobs[max].top;
    blobs.blobs[min].bottom = (blobs.blobs[min].bottom>blobs.blobs[max].bottom) ? blobs.blobs[min].bottom : blobs.blobs[max].bottom;
    blobs.blobs[min].left = (blobs.blobs[min].left<blobs.blobs[max].left) ? blobs.blobs[min].left : blobs.blobs[max].left;
    blobs.blobs[min].right = (blobs.blobs[min].right>blobs.blobs[max].right) ? blobs.blobs[min].right : blobs.blobs[max].right;
    blobs.blobs[min].x = (blobs.blobs[min].left+blobs.blobs[min].right)/2;
    blobs.blobs[min].y = (blobs.blobs[min].top+blobs.blobs[min].bottom)/2;
    blobs.blobs[min].area = (blobs.blobs[min].bottom-blobs.blobs[min].top)*(blobs.blobs[min].right-blobs.blobs[min].left);

    blobs.blobs.erase(blobs.blobs.begin()+max);
    blobs.blob_count = blobs.blobs.size();
    return min;
}

bool blobsAreAdjacent(const unsigned int &b1, const unsigned int &b2, const cmvision::Blobs &blobs)
{
    return ((abs((int)(blobs.blobs[b1].x-blobs.blobs[b2].x))-BLOB_ADJACENT_THRESH <= (int)((blobs.blobs[b1].right-blobs.blobs[b1].left)/2+(blobs.blobs[b2].right-blobs.blobs[b2].left)/2)) &&
            (abs((int)(blobs.blobs[b1].y-blobs.blobs[b2].y))-BLOB_ADJACENT_THRESH <= (int)((blobs.blobs[b1].bottom-blobs.blobs[b1].top)/2+(blobs.blobs[b2].bottom-blobs.blobs[b2].top)/2)));
}

bool blobsAreSameColor(const unsigned int &b1, const unsigned int &b2, const cmvision::Blobs &blobs)
{
    return (blobs.blobs[b1].red==blobs.blobs[b2].red && blobs.blobs[b1].green==blobs.blobs[b2].green && blobs.blobs[b1].blue==blobs.blobs[b2].blue);
}

int findSimilarAdjacentBlob(const unsigned int &b1, cmvision::Blobs &blobs)
{
    for(int i = 0;(unsigned int)i < blobs.blob_count;i++)
    {
        if(i != (int)b1 && blobsAreSameColor(b1,i,blobs) && blobsAreAdjacent(b1,i,blobs)) return i;
    }
    return -1;
}

float blobAngle(const unsigned int &b1, const unsigned int &b2, const cmvision::Blobs &blobs)
{
    if(b1==b2 || b1 >= blobs.blobs.size() || b2 >= blobs.blobs.size()) return 0;
    return (atan2(blobs.blobs[b2].y-blobs.blobs[b1].y,blobs.blobs[b2].x-blobs.blobs[b1].x));
}

int findAdjacentBlobs(const unsigned int &b1, const cmvision::Blobs &blobs, std::vector<unsigned int> &adj_blobs)
{
    int begin_size = adj_blobs.size();
    for(unsigned int i = 0;i<blobs.blob_count;i++)
    {
        if(!isInList(i,adj_blobs) && blobsAreAdjacent(b1,i,blobs)) adj_blobs.push_back(i); //std::cout << "adding a new blob" << std::endl;}
    }
    return begin_size-adj_blobs.size();
}

void deleteBlob(const unsigned int &b1, cmvision::Blobs &blobs)
{
    if(b1 < blobs.blobs.size())
    {
        //std::cout << "deleting blob " << b1 << std::endl;
        blobs.blobs.erase(blobs.blobs.begin()+b1);
        blobs.blob_count--;
    }
}

int findBlobCluster(const unsigned int &b1, const cmvision::Blobs &blobs, std::vector<unsigned int> &blob_cluster)
{
    blob_cluster.empty();
    std::vector<unsigned int> new_cluster;
    
    findAdjacentBlobs(b1,blobs,blob_cluster);
    
    for(int i = 0;(unsigned int)i < blob_cluster.size();i++) 
    {
        findAdjacentBlobs(blob_cluster[i],blobs,blob_cluster);
    }
    return blob_cluster.size();
}

void orderCluster(std::vector<unsigned int>& cluster,cmvision::Blobs blobs)
{
    unsigned int tmp;
    for(unsigned int i=0;i<cluster.size();i++)
    {
        for(unsigned int j=0;j<cluster.size()-1-i;j++)
        {
            if(blobs.blobs[cluster[j]].x>blobs.blobs[cluster[j+1]].x)
            {
                tmp = cluster[j];
                cluster[j] = cluster[j+1];
                cluster[j+1] = tmp;
            }
        }
    }
}

cmvision::Blob& getBlob(const unsigned int &blob_num, cmvision::Blobs &blobs)
{
    return blobs.blobs[blob_num];
}

void printBinary(char n)
{
    unsigned int i;
    i = 1<<(sizeof(n) * 8 - 1);

    while (i > 0) {
        if (n & i)
            printf("1");
        else
            printf("0");
        i >>= 1;
    }
}

char getColorID(unsigned int blob_num,cmvision::Blobs& blobs)
{
    return ((getBlob(blob_num,blobs).red!=0) << 1)|((getBlob(blob_num,blobs).green!=0)<<0);
}

void blobSequenceFromCluster(coax_client::BlobSequence &blob_sequence, std::vector<unsigned int> &cluster, cmvision::Blobs blobs)
{ 
    orderCluster(cluster,blobs);
    for(unsigned int i = 0;i<cluster.size();i++)
    {
        blob_sequence.id |= (getColorID(cluster[i],blobs) << (3-i)*2);
        blob_sequence.x += getBlob(cluster[i],blobs).x;
        blob_sequence.y += getBlob(cluster[i],blobs).y;
    }
    blob_sequence.sequence = cluster;
    blob_sequence.x /= cluster.size();
    blob_sequence.y /= cluster.size();
}

int findAllBlobClusters(cmvision::Blobs blobs, std::vector<std::vector<unsigned int> > &blob_clusters)
{
    std::list<unsigned int> have_no_cluster(blobs.blobs.size(),0);
    int i = 1;
    for(std::list<unsigned int>::iterator pos = ++have_no_cluster.begin();pos != have_no_cluster.end();++pos)
    {
        *pos = i;
        i++;
    }
    
    while(have_no_cluster.size())
    {
        //cout << "blobs with no cluster, finding cluster." << endl;
        std::vector<unsigned int> cluster;
        findBlobCluster(have_no_cluster.front(),blobs,cluster);
        for(int i = 0;i<(int)cluster.size();i++)
            have_no_cluster.remove(cluster[i]);
        blob_clusters.push_back(cluster);
    }
    
    return blob_clusters.size();
}

#endif
