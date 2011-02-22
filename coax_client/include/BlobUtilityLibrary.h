/**
 * @file: BlobUtilityLibrary.h
 * @date: 2/17/2011
 * @auther: Aaron Parker
 */

#ifndef BLOB_UTILITY_LIBRARY_H
#define BLOB_UTILITY_LIBRARY_H

#include <CoaxClientConst.h>
#include <cmvision/Blobs.h>
#include <ros/ros.h>

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

int findAllBlobClusters(cmvision::Blobs blobs, std::vector<std::vector<unsigned int> > &blob_clusters)
{
    std::vector<bool> blob_is_clustered(blobs.blobs.size(),false);
    blob_clusters.empty();
    
    unsigned int next = 0;
    
    while(blobs.blob_count)
    {
        std::vector<unsigned int> blob_cluster;
        findBlobCluster(next,blobs,blob_cluster);
        blobs.blob_count -= blob_cluster.size();
        blob_clusters.push_back(blob_cluster);
    }
    
    return blob_clusters.size();
}

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
        std::cout << "deleting blob " << b1 << std::endl;
        blobs.blobs.erase(blobs.blobs.begin()+b1);
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

#endif
