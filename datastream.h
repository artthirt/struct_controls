#ifndef DATASTREAM_H
#define DATASTREAM_H

#include <vector>
#include <string>
#include <assert.h>

class basicstream{
public:
	/**
	 * @brief readRawData
	 * @param data
	 * @param len
	 * @return
	 */
	virtual int readRawData(char* data, int len) {}
	/**
	 * @brief writeRawData
	 * @param data
	 * @param len
	 * @return
	 */
	virtual int writeRawData(char* data, int len) {}
	/**
	 * @brief pos
	 * @return
	 */
	inline int pos() const {return m_pos;}

protected:
	/**
	 * @brief inc
	 * @param len
	 */
	inline void inc(int len){ m_pos += len; }
	/**
	 * @brief set_pos
	 * @param pos
	 */
	inline void set_pos(int pos){ m_pos = pos; }

private:
	int m_pos;

};

class inputstream: public basicstream{
public:
	inputstream(std::vector< char >* source);
	/**
	 * @brief write
	 * @param v
	 */
	template< typename T >
	void write(const T& v){
		int sizetype = sizeof(v);
		if(pos() + sizetype > m_buffer->size()){
			m_buffer->resize(pos() + sizetype);
		}
		std::copy((char*)&v, (char*)&v + sizetype, &(*m_buffer)[pos()]);
		inc(sizetype);
	}
	/**
	 * @brief writeRawData
	 * @param data
	 * @param len
	 * @return
	 */
	virtual int writeRawData(char* data, int len);
private:
	std::vector< char > *m_buffer;
};

class outputstream: public basicstream{
public:
	outputstream(const std::vector< char > &source);
	/**
	 * @brief read
	 * @return
	 */
	template< typename T >
	T read(){
		T v(0);
		int sizetype = sizeof(v);
		if(m_buffer.size() < pos() + sizetype)
			return v;
		std::copy((char*)&m_buffer[pos()], (char*)&m_buffer[pos()] + sizetype, (char*)&v);
		inc(sizetype);
		return v;
	}
	/**
	 * @brief readRawData
	 * @param data
	 * @param len
	 * @return
	 */
	virtual int readRawData(char* data, int len);
private:
	std::vector< char > m_buffer;
};

class datastream
{
public:
	datastream(const std::vector< char > &source);
	datastream(std::vector< char > *source);
	~datastream();

	/**
	 * @brief operator >>
	 * @param v
	 * @return
	 */
	template< typename T >
	inline datastream& operator>> (T& v){
		v = T();
		outputstream* os = dynamic_cast< outputstream* >(m_stream);
		assert(os != 0);
		if(os)
			v = os->read<T>();
		return *this;
	}

	/**
	 * @brief operator <<
	 * @param v
	 * @return
	 */
	template< typename T >
	inline datastream& operator<< (T v){
		inputstream *is = dynamic_cast< inputstream* >(m_stream);
		assert(is != 0);
		if(is)
			is->write(v);
		return *this;
	}
	/**
	 * @brief readRawData
	 * @param data
	 * @param len
	 * @return
	 */
	int readRawData(char* data, int len);
	/**
	 * @brief writeRawData
	 * @param data
	 * @param len
	 * @return
	 */
	int writeRawData(char* data, int len);
private:
	basicstream *m_stream;
};

#endif // DATASTREAM_H
