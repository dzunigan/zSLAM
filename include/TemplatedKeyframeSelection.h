/**
* This file is part of zSLAM.
*
* Copyright (c) 2016 D. Zúñiga Noël
* For more information see <https://github.com/Peski/zSLAM>
*/

#ifndef Z_TEMPLATED_KEYFRAME_SELECTION_H
#define Z_TEMPLATED_KEYFRAME_SELECTION_H

#include <DBoW2/TemplatedVocabulary.h>

#include <memory>

template<class TDescriptor, class F>
class TemplatedKeyframeSelection
{
public:

    template<class T>
    explicit TemplatedKeyframeSelection(const T &voc, float beta);

    /**
     * Destructor
     */
    virtual ~TemplatedKeyframeSelection(void);

    template<class T>
    inline void setVocabulary(const T &voc);

    //TODO: return computed BoW
    bool process(const vector<TDescriptor> &features);

    inline void reset();

protected:

    /// Associated vocabulary
    DBoW2::TemplatedVocabulary<TDescriptor, F> *m_voc;

    /// Threshold
    float m_beta;

    bool m_initialized;

    //TODO: boost::shared_ptr
    std::shared_ptr<DBoW2::BowVector> m_last;
    std::shared_ptr<DBoW2::BowVector> m_prev;
    std::shared_ptr<DBoW2::BowVector> m_vec;
};

template<class TDescriptor, class F>
template<class T>
TemplatedKeyframeSelection<TDescriptor, F>::TemplatedKeyframeSelection(const T &voc, float beta)
    : m_voc(NULL), m_beta(beta), m_last(nullptr), m_prev(nullptr), m_vec(nullptr)
{
    setVocabulary(voc);
}

template<class TDescriptor, class F>
TemplatedKeyframeSelection<TDescriptor, F>::~TemplatedKeyframeSelection(void)
{
    delete m_voc;
}

template<class TDescriptor, class F>
template<class T>
inline void TemplatedKeyframeSelection<TDescriptor, F>::setVocabulary(const T& voc)
{
    m_voc = new T(voc);
    reset();
}

template<class TDescriptor, class F>
bool TemplatedKeyframeSelection<TDescriptor, F>::process(const vector<TDescriptor> &features)
{
    m_vec = std::make_shared<DBoW2::BowVector>();
    m_voc->transform(features, *m_vec);

    bool ret;
    if (m_initialized)
    {
        double factor = m_voc->score(*m_prev, *m_vec);
        //TODO: if factor < ...
        ret = (m_voc->score(*m_last, *m_vec) / factor) < m_beta;

        m_prev = m_vec;

        if (ret)
            m_last = m_vec;
    }
    else
    {
        m_prev = m_vec;
        m_last = m_vec;
        m_initialized = true;
        ret = false;
    }

    return ret;
}

template<class TDescriptor, class F>
inline void TemplatedKeyframeSelection<TDescriptor, F>::reset()
{
    m_initialized = false;
}

#endif
